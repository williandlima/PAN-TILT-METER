"""Fonte de dados real: leitura do ângulo via Modbus RTU, pela porta serial
USB do ESP32 (conectado direto por cabo — sem RS485).

Usa o kwarg `device_id=` do pymodbus (renomeado de `slave=` nas versões mais
recentes da lib) — por isso `requirements.txt` trava `pymodbus==3.14.0`
exatamente: sem essa trava, um `pip install` puxando uma versão diferente
pode voltar a quebrar com `unexpected keyword argument`.

Contrato com o firmware do ESP32: a inclinação (tilt) é exposta no
registrador de entrada (input register) `ANGLE_INPUT_REGISTER` e o azimute
(pan) em `PAN_INPUT_REGISTER`, ambos como inteiro de 16 bits **com sinal**
igual a `angulo * SCALE` (duas casas decimais de resolução). Logo depois vêm
os quatro registradores de extremo (mín/máx de cada eixo), na mesma
codificação. Os seis são contíguos de propósito: saem numa transação só.

Os extremos são medidos pelo firmware, e não pelo app, desde a v1.5.0. A
razão é de amostragem: o firmware vê o sensor a 100 Hz, este poll roda a
4 Hz, e uma rajada de vento de meio segundo — justamente o que o relatório
existe para registrar — acontece inteira entre duas leituras daqui. Ver o
bloco ANGLE_PEAK_* em firmware/src/Config.h. Contra firmware antigo o app
volta a calcular os extremos por conta própria (`limits.limit_tracker`).

Compatibilidade com firmware antigo: um firmware que não conhece um
registrador responde com exceção de "endereço inválido" e derruba a leitura
inteira junto. Por isso a leitura começa pedindo os seis registradores e, a
cada *exceção Modbus*, encolhe o pedido de forma permanente — seis (com
extremos, v1.5.0+), dois (só os dois eixos, v1.2.0+) ou um (só tilt). A
degradação é só para exceção do escravo: erro de transporte (timeout, CRC)
não desabilita nada, senão uma falha passageira de cabo deixaria recursos
desligados pelo resto da sessão.

Calibração: escrever `True` na coil `CALIBRATE_COIL` sinaliza ao firmware
para zerar **os dois eixos** na posição atual (novo "zero" mecânico), o que
zera os extremos junto. Para esquecer só os extremos, sem mexer no zero,
escreve-se `True` em `RESET_PEAKS_COIL`.

Captura de vibração (leitura em alta taxa, usada para caracterizar variação
angular por vento/vibração — o poll normal de ~250ms é lento demais para
isso): contrato assumido, a confirmar quando o firmware existir.
- App escreve `duration_s` em `VIBRATION_DURATION_REG` e `rate_hz` em
  `VIBRATION_RATE_REG`, depois escreve `True` na coil `VIBRATION_START_COIL`
  para iniciar. O firmware passa a amostrar internamente na taxa pedida e
  guarda o buffer em memória.
- App faz poll de `VIBRATION_STATUS_REG` (0=ocioso, 1=capturando, 2=pronto,
  3=erro), `VIBRATION_PROGRESS_REG` (0-100) e `VIBRATION_SAMPLE_COUNT_REG`
  (total de amostras, válido quando status=pronto).
- Quando pronto, app lê o buffer em blocos: escreve o índice inicial em
  `VIBRATION_CURSOR_REG`, depois lê `VIBRATION_BLOCK_SIZE` registradores a
  partir de `VIBRATION_BLOCK_START_REG` (cada um = ângulo relativo ao "zero"
  calibrado * `ANGLE_SCALE`, inteiro **com sinal** de 16 bits, já que a
  variação pode ser negativa) — repete até completar `VIBRATION_SAMPLE_COUNT_REG`.
- O eixo de **pan** vem no mesmo esquema, a partir de
  `VIBRATION_PAN_BLOCK_START_REG`, com o mesmo cursor. A grandeza ali é
  diferente: **velocidade angular em graus/s** * `PAN_RATE_SCALE`, e não
  ângulo (ver `firmware/src/VibrationCapture.h` para o porquê). A conversão
  para ângulo é feita por `limits.vibration_stats.pan_rates_to_angles`.
  Firmware anterior à v1.3.0 rejeita esses registradores com exceção de
  endereço inválido — nesse caso a captura sai só com o tilt.
"""
from __future__ import annotations

import threading
import time
from typing import NamedTuple

from data_source.base import (
    AngleReading,
    ErrorCallback,
    IAngleDataSource,
    ReadingCallback,
    build_vibration_readings,
)

# Muitas placas ESP32 (incluindo a usada neste projeto, ESP32-DevKitC V4
# com chip CP2102N) resetam a placa via DTR/RTS toda vez que a porta
# serial é aberta — é o mesmo mecanismo de auto-reset usado para gravar o
# firmware sem apertar botão. O app abre a porta com as duas linhas
# desligadas para evitar isso
# (ver `_make_client`), e sonda a placa até ela responder em vez de esperar
# um tempo fixo (ver BOARD_READY_TIMEOUT_S), para o caso de o driver do
# chip USB-serial pulsar as linhas mesmo assim.

# Quantas vezes tentar abrir a porta antes de desistir, e quanto esperar
# entre tentativas. Existe porque o sistema operacional pode levar um
# instante para liberar de verdade uma porta serial que acabou de ser
# fechada por outra conexão (ex: o teste de "Detectar automaticamente" ou
# "Testar conexão" rodando poucos segundos antes de clicar em "Iniciar") —
# sem isso, `connect()` falha com a porta ainda "ocupada" mesmo com o
# hardware saudável, e a única saída era esperar e tentar novamente à mão.
CONNECT_RETRY_ATTEMPTS = 3
CONNECT_RETRY_DELAY_S = 1.0

# Quantas leituras seguidas precisam falhar antes de o app fechar a porta e
# reabri-la. Existe porque uma falha de leitura persistente (cabo USB
# removido e recolocado, ESP32 resetado, driver que largou a porta) deixava a
# leitura contínua num estado sem saída: o laço abaixo seguia tentando ler de
# um cliente morto para sempre, repetindo o mesmo erro a cada 250ms, e a
# única saída era o usuário parar e iniciar de novo na mão. Reconectar
# recupera sozinho assim que o hardware volta.
#
# Cinco falhas (~1,25s no poll padrão) é tolerante o bastante para não
# reconectar por causa de um timeout isolado — reabrir a porta reseta o
# ESP32 quando o driver pulsa DTR/RTS, então não é uma operação barata.
READ_FAILURES_BEFORE_RECONNECT = 5

# Quantas reconexões seguidas tentar antes de desistir e deixar a leitura
# parada com uma mensagem clara. O contador zera assim que uma leitura volta a
# dar certo, então uma falha ocasional depois de horas de operação não consome
# o orçamento da próxima. Sem esse teto, um ESP32 que nunca mais responde
# (mas cuja porta serial continua existindo) deixaria o app reabrindo a porta
# para sempre — e cada reabertura reseta a placa.
RECONNECT_ATTEMPTS = 5

ANGLE_INPUT_REGISTER = 0
PAN_INPUT_REGISTER = 1  # contíguo ao de tilt de propósito: os dois saem numa leitura só
# Extremos medidos pelo firmware, contíguos aos dois acima (registradores 2 a
# 5): tilt mín, tilt máx, pan mín, pan máx.
ANGLE_MIN_INPUT_REGISTER = 2
PEAK_REGISTER_COUNT = 6  # os seis registradores lidos numa transação só
ANGLE_SCALE = 100.0  # registrador = ângulo * 100 (int16, resolução de 0.01°)
CALIBRATE_COIL = 0
RESET_PEAKS_COIL = 2  # esquece os extremos sem mexer no zero
FIRMWARE_VERSION_REGISTER = 40  # registrador = major*10000 + minor*100 + patch (ex: "1.0.0" -> 10000)

VIBRATION_START_COIL = 1
VIBRATION_DURATION_REG = 10  # segundos (uint16)
VIBRATION_RATE_REG = 11  # amostras/s (uint16)
VIBRATION_STATUS_REG = 20  # 0=ocioso, 1=capturando, 2=pronto, 3=erro
VIBRATION_PROGRESS_REG = 21  # percentual 0-100
VIBRATION_SAMPLE_COUNT_REG = 22  # total de amostras capturadas (válido quando status=pronto)
VIBRATION_CURSOR_REG = 30  # índice inicial do próximo bloco a ler (escrita) — vale para os dois eixos
VIBRATION_BLOCK_START_REG = 31  # início do bloco de amostras de TILT (leitura)
VIBRATION_BLOCK_SIZE = 32  # amostras por bloco de leitura
VIBRATION_PAN_BLOCK_START_REG = 70  # início do bloco de amostras de PAN (taxa em graus/s * PAN_RATE_SCALE)
PAN_RATE_SCALE = 100.0  # registrador = graus/s * 100 (int16, faixa +-327°/s)
VIBRATION_STATUS_POLL_INTERVAL_S = 0.5


def _to_signed16(raw: int) -> int:
    return raw - 0x10000 if raw >= 0x8000 else raw


def _decode_firmware_version(code: int) -> str:
    major = code // 10000
    minor = (code // 100) % 100
    patch = code % 100
    return f"{major}.{minor}.{patch}"


def _is_slave_exception(result) -> bool:
    """True se o escravo respondeu com uma exceção Modbus (ex: endereço
    inválido), em vez de a transação ter falhado no transporte (timeout, CRC).

    Checa o atributo em vez de importar `ExceptionResponse` porque o caminho
    de import dessa classe já mudou entre versões do pymodbus.
    """
    return getattr(result, "exception_code", None) is not None


# O app usa a API do pymodbus 3.10+ (argumento `device_id`, que substituiu
# `slave`). Com uma versão mais antiga instalada no PC, TODA chamada falhava
# com um TypeError que aparecia como "erro de leitura" genérico.
PYMODBUS_MIN_VERSION = (3, 10)

# Espera máxima, depois de abrir a porta, até o ESP32 responder. Se a
# abertura reiniciou a placa (auto-reset por DTR/RTS), ela volta em ~1-2 s;
# se não reiniciou, responde na hora. Sondar em vez de dormir um tempo fixo
# serve aos dois casos.
BOARD_READY_TIMEOUT_S = 6.0
BOARD_READY_POLL_S = 0.25

# Timeout de resposta e repetições internas do pymodbus. A 9600 baud a maior
# resposta do firmware (bloco de 32 amostras, 69 bytes) leva ~72 ms, e o
# firmware responde no mesmo ciclo do loop — 0,3 s é folga de sobra. Com os
# padrões do pymodbus (3 s, ou 1 s daqui, x 3 repetições), cada falha
# isolada congelava a leitura por 4 s.
RESPONSE_TIMEOUT_S = 0.3
PYMODBUS_RETRIES = 1


def _require_pymodbus() -> None:
    import pymodbus

    parts = []
    for piece in pymodbus.__version__.split(".")[:2]:
        digits = "".join(ch for ch in piece if ch.isdigit())
        parts.append(int(digits or 0))
    if tuple(parts) < PYMODBUS_MIN_VERSION:
        raise RuntimeError(
            f"pymodbus {pymodbus.__version__} instalado, mas o app precisa do 3.14 — "
            f"rode: pip install -r requirements.txt"
        )


def _make_client(port: str, baudrate: int, timeout_s: float, retries: int = PYMODBUS_RETRIES):
    """Cliente Modbus RTU que abre a porta SEM reiniciar o ESP32.

    O circuito de auto-reset das placas ESP32 DevKit liga DTR/RTS do chip
    USB-serial aos pinos EN/IO0 — é assim que o PlatformIO grava sem apertar
    botão. O pyserial abre a porta com DTR e RTS ativos, o que reinicia a
    placa a cada abertura: calibração, extremos e referência do pan se
    perdiam em todo "Testar conexão", "Iniciar" ou reconexão. Abrindo com as
    duas linhas já desligadas (estado de repouso do circuito, placa rodando
    normalmente), o pulso de reset não acontece. Depende do driver do chip
    USB-serial; quando ele pulsar as linhas mesmo assim, a sondagem em
    `_wait_until_ready` absorve o reboot.
    """
    import serial
    from pymodbus.client import ModbusSerialClient

    class _NoResetSerialClient(ModbusSerialClient):
        def connect(self) -> bool:
            if self.socket:
                return True
            try:
                s = serial.Serial()
                s.port = self.comm_params.host
                s.baudrate = self.comm_params.baudrate
                s.bytesize = self.comm_params.bytesize
                s.parity = self.comm_params.parity
                s.stopbits = self.comm_params.stopbits
                s.timeout = self.comm_params.timeout_connect
                s.dtr = False
                s.rts = False
                s.open()
                s.inter_byte_timeout = self.inter_byte_timeout
                self.socket = s
            except Exception:  # noqa: BLE001 - mesmo contrato do connect() original
                self.close()
                if not hasattr(self, "comm_params"):
                    return super().connect()
            return self.socket is not None

    client = _NoResetSerialClient(port=port, baudrate=baudrate, timeout=timeout_s, retries=retries)
    # O pymodbus 3.x fecha a porta sozinho depois de retries+3 transações
    # seguidas sem resposta, e a reabre na chamada seguinte. Numa placa que
    # reinicia ao abrir a porta, a sondagem durante o boot passava desse
    # limite, a reabertura reiniciava a placa de novo — e o ciclo se
    # repetia. Quem decide reconectar é o app (READ_FAILURES_BEFORE_RECONNECT).
    transaction = getattr(client, "transaction", None)
    if transaction is not None and hasattr(transaction, "max_until_disconnect"):
        transaction.max_until_disconnect = transaction.count_until_disconnect = 10**9
    return client


def _wait_until_ready(client, slave_id: int, max_wait_s: float = BOARD_READY_TIMEOUT_S) -> bool:
    """Sonda o escravo até ele responder (ver BOARD_READY_TIMEOUT_S).

    No pymodbus 3.x, falta de resposta LEVANTA ModbusIOException (não volta
    como resultado com isError()) — daí o try.
    """
    deadline = time.monotonic() + max_wait_s
    while True:
        try:
            result = client.read_input_registers(address=ANGLE_INPUT_REGISTER, count=1, device_id=slave_id)
            if not result.isError() or _is_slave_exception(result):
                return True
        except Exception:  # noqa: BLE001 - sem resposta ainda (placa reiniciando)
            pass
        if time.monotonic() >= deadline:
            return False
        time.sleep(BOARD_READY_POLL_S)


# Tentativas por transação durante a captura de vibração (ver `_transact`).
VIBRATION_TRANSACTION_ATTEMPTS = 5
VIBRATION_TRANSACTION_RETRY_DELAY_S = 0.1


class _SlaveRejected(Exception):
    """O escravo respondeu com exceção Modbus (endereço inexistente)."""


class _CaptureCancelled(Exception):
    """Captura cancelada pelo usuário durante uma transação."""


class SlaveCapabilities:
    """O que o firmware conectado sabe responder.

    Cada flag cai para `False` de forma permanente quando o escravo rejeita o
    bloco correspondente com uma exceção Modbus — ver "Compatibilidade com
    firmware antigo" no cabeçalho do módulo. Fica numa instância mutável (e
    não em flags devolvidas pela função) para o estado ser um só, mesmo com a
    leitura sendo chamada de lugares diferentes.
    """

    def __init__(self) -> None:
        self.pan = True
        self.peaks = True


class AxisSample(NamedTuple):
    """Uma leitura dos eixos, com os extremos quando o firmware os fornece."""

    angle_deg: float
    pan_deg: float | None = None
    angle_min_deg: float | None = None
    angle_max_deg: float | None = None
    pan_min_deg: float | None = None
    pan_max_deg: float | None = None


def _read_axes(client, slave_id: int, caps: SlaveCapabilities) -> AxisSample:
    """Lê os eixos do escravo, com os extremos quando disponíveis.

    Encolhe o pedido conforme o escravo rejeita registradores, atualizando
    `caps`. Levanta `IOError` em falha de transporte.
    """
    if caps.peaks:
        result = client.read_input_registers(
            address=ANGLE_INPUT_REGISTER, count=PEAK_REGISTER_COUNT, device_id=slave_id
        )
        if not result.isError():
            values = [_to_signed16(raw) / ANGLE_SCALE for raw in result.registers]
            return AxisSample(*values)
        if not _is_slave_exception(result):
            raise IOError(str(result))
        caps.peaks = False  # firmware anterior à v1.5.0: extremos ficam por conta do app

    if caps.pan:
        result = client.read_input_registers(
            address=ANGLE_INPUT_REGISTER, count=2, device_id=slave_id
        )
        if not result.isError():
            return AxisSample(
                _to_signed16(result.registers[0]) / ANGLE_SCALE,
                _to_signed16(result.registers[1]) / ANGLE_SCALE,
            )
        if not _is_slave_exception(result):
            raise IOError(str(result))
        caps.pan = False  # firmware sem eixo de pan: segue só com o tilt

    result = client.read_input_registers(address=ANGLE_INPUT_REGISTER, count=1, device_id=slave_id)
    if result.isError():
        raise IOError(str(result))
    return AxisSample(_to_signed16(result.registers[0]) / ANGLE_SCALE)


class ConnectionTestResult(NamedTuple):
    angle_deg: float
    firmware_version: str
    pan_deg: float | None = None


# Tolerância do teste de conexão. O teste precisa ser tão tolerante quanto a
# leitura contínua (que aceita READ_FAILURES_BEFORE_RECONNECT falhas seguidas
# e reabre a porta): mais rígido que ela, ele reprovava uma placa que a
# leitura normal usava sem problema — o caso em que a primeira resposta logo
# depois do reset (que a abertura da porta provoca) não chega. Cada tentativa
# é curta (sem os retries internos do pymodbus) para o diálogo não ficar
# congelado muito tempo.
TEST_READ_ATTEMPTS = 4
TEST_READ_RETRY_DELAY_S = 0.5
TEST_OPEN_CYCLES = 2


def test_connection(
    port: str,
    baudrate: int,
    slave_id: int,
    timeout_s: float = RESPONSE_TIMEOUT_S,
    open_cycles: int = TEST_OPEN_CYCLES,
) -> ConnectionTestResult:
    """Testa a conexão Modbus RTU (via USB) com o ESP32: abre a porta, lê o
    ângulo e a versão do firmware, e fecha a conexão. Tenta a leitura
    TEST_READ_ATTEMPTS vezes e, se nenhuma responder, reabre a porta (novo
    reset da placa) até `open_cycles` vezes — ver TEST_READ_ATTEMPTS. Levanta
    exceção (IOError/RuntimeError) se nada der certo.
    """
    _require_pymodbus()
    last_error: Exception | None = None
    for _cycle in range(open_cycles):
        client = _make_client(port, baudrate, timeout_s, retries=0)
        try:
            connected = False
            for attempt in range(CONNECT_RETRY_ATTEMPTS):
                if client.connect():
                    connected = True
                    break
                if attempt < CONNECT_RETRY_ATTEMPTS - 1:
                    time.sleep(CONNECT_RETRY_DELAY_S)
            if not connected:
                raise IOError(f"Não foi possível abrir a porta serial {port}.")
            if not _wait_until_ready(client, slave_id):
                last_error = IOError("sem resposta do ESP32")
                continue  # nem a sondagem respondeu: reabre direto

            sample = None
            for attempt in range(TEST_READ_ATTEMPTS):
                try:
                    sample = _read_axes(client, slave_id, SlaveCapabilities())
                    break
                except Exception as exc:  # noqa: BLE001 - tenta de novo
                    last_error = exc
                    if attempt < TEST_READ_ATTEMPTS - 1:
                        time.sleep(TEST_READ_RETRY_DELAY_S)
            if sample is None:
                continue  # reabre a porta (novo reset da placa)

            # A placa já respondeu; a versão é diagnóstico secundário e não
            # reprova o teste — mas uma resposta perdida aqui não pode virar
            # exceção (no pymodbus 3.x, falta de resposta levanta).
            firmware_version = "?"
            for attempt in range(TEST_READ_ATTEMPTS):
                try:
                    version_result = client.read_input_registers(
                        address=FIRMWARE_VERSION_REGISTER, count=1, device_id=slave_id
                    )
                    if not version_result.isError():
                        firmware_version = _decode_firmware_version(version_result.registers[0])
                        break
                    if _is_slave_exception(version_result):
                        break  # firmware sem o registrador de versão
                except Exception:  # noqa: BLE001 - tenta de novo
                    pass
                time.sleep(TEST_READ_RETRY_DELAY_S)
            return ConnectionTestResult(sample.angle_deg, firmware_version, sample.pan_deg)
        finally:
            client.close()
    raise IOError(f"O ESP32 não respondeu na porta {port}: {last_error}")


# Teto de tempo para testar UMA porta, usado por `_probe_port` abaixo.
# Generoso o bastante para cobrir BOARD_READY_TIMEOUT_S + as leituras reais
# (com folga), mas existe sobretudo para as portas erradas: sem ele, uma
# porta "fantasma" (ver `_looks_like_bluetooth_port`) pode travar a busca
# inteira por dezenas de segundos ou mais.
PORT_PROBE_TIMEOUT_S = 12.0


def _looks_like_bluetooth_port(port_info) -> bool:
    """Portas seriais virtuais criadas pela pilha Bluetooth do Windows (ex:
    "Standard Serial over Bluetooth link", uma para cada dispositivo já
    pareado, mesmo sem estar por perto) aparecem em `list_ports.comports()`
    junto com portas USB de verdade. Não são o ESP32, e tentar abri-las é a
    causa mais comum de a detecção automática travar por muito tempo — em
    alguns casos o próprio `open()` do sistema operacional demora dezenas de
    segundos pra desistir de um dispositivo pareado que não está conectado,
    o que nenhum parâmetro de timeout do pyserial/pymodbus consegue limitar
    (o bloqueio acontece antes, na camada do SO). Descartadas aqui, antes de
    sequer tentar abrir — mais rápido e mais confiável do que só confiar no
    teto de tempo de `_probe_port`."""
    haystack = " ".join(
        str(v) for v in (getattr(port_info, "description", None), getattr(port_info, "hwid", None)) if v
    ).lower()
    return "bluetooth" in haystack


def _probe_port(port: str, baudrate: int, slave_id: int, timeout_s: float) -> bool:
    """Testa uma porta com um teto de tempo total (`PORT_PROBE_TIMEOUT_S`),
    porque uma porta problemática pode travar antes mesmo do timeout que o
    pymodbus/pyserial aplicam às leituras — o bloqueio acontece no próprio
    `open()` da porta, numa chamada de sistema que o Python não tem como
    interromper de fora. Roda a tentativa numa thread separada e, se ela não
    voltar a tempo, desiste e segue para a próxima porta, deixando a thread
    travada morrer sozinha em segundo plano (é `daemon`, não impede o app de
    fechar)."""
    result: list[bool] = [False]

    def attempt() -> None:
        try:
            # Um ciclo só: cada porta errada já custa o teto inteiro.
            test_connection(port, baudrate, slave_id, timeout_s=timeout_s, open_cycles=1)
            result[0] = True
        except Exception:  # noqa: BLE001 - porta errada, ou nada conectado nela
            result[0] = False

    thread = threading.Thread(target=attempt, daemon=True)
    thread.start()
    thread.join(PORT_PROBE_TIMEOUT_S)
    return result[0]


def find_port(baudrate: int, slave_id: int, timeout_s: float = RESPONSE_TIMEOUT_S) -> str | None:
    """Varre as portas seriais do sistema em busca do ESP32, testando cada
    uma de verdade com `test_connection` (não dá pra confiar só em VID/PID:
    o chip USB-serial do hardware confirmado, CP2102N, tem um VID:PID de
    fábrica estável (`10C4:EA60`), mas é o padrão genérico da Silicon Labs
    — compartilhado por inúmeros outros dispositivos USB-serial não
    relacionados a este projeto, então filtrar por ele arriscaria tanto
    ignorar a porta certa (driver/SO reportando algo diferente) quanto
    aceitar a porta errada) — exceto as que claramente não são candidatas,
    como as portas Bluetooth virtuais (ver `_looks_like_bluetooth_port`).

    Devolve o nome da primeira porta que responder como o ESP32, ou `None`
    se nenhuma responder. Cada porta errada custa até `PORT_PROBE_TIMEOUT_S`
    de espera; a porta certa custa até `BOARD_READY_TIMEOUT_S` (o reset
    que a abertura da porta provoca no ESP32) — por isso esta função é
    pensada para rodar numa thread de fundo, não na UI.
    """
    from serial.tools import list_ports

    for p in list_ports.comports():
        if _looks_like_bluetooth_port(p):
            continue
        if _probe_port(p.device, baudrate, slave_id, timeout_s):
            return p.device
    return None


class ModbusAngleSource(IAngleDataSource):
    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        slave_id: int = 1,
        poll_interval_s: float = 0.25,
        timeout_s: float = RESPONSE_TIMEOUT_S,
    ) -> None:
        self._port = port
        self._baudrate = baudrate
        self._slave_id = slave_id
        self._poll_interval = poll_interval_s
        self._timeout = timeout_s

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

        self._client_lock = threading.Lock()
        self._client = None

        self._vibration_thread: threading.Thread | None = None
        self._vibration_stop_event = threading.Event()

        # Encolhe conforme o escravo rejeita registradores — ver o cabeçalho
        # do módulo.
        self._caps = SlaveCapabilities()
        self._firmware_version: str | None = None

    @property
    def label(self) -> str:
        return f"USB/Modbus RTU ({self._port}@{self._baudrate}, id={self._slave_id})"

    @property
    def firmware_version(self) -> str | None:
        return self._firmware_version

    def _read_firmware_version(self, client) -> None:
        """Lê a versão do firmware (registrador 40) logo após conectar. Só
        diagnóstico: uma falha aqui não interrompe a leitura."""
        for _ in range(3):
            try:
                result = client.read_input_registers(
                    address=FIRMWARE_VERSION_REGISTER, count=1, device_id=self._slave_id
                )
                if not result.isError():
                    self._firmware_version = _decode_firmware_version(result.registers[0])
                    return
                if _is_slave_exception(result):
                    return  # firmware sem o registrador de versão
            except Exception:  # noqa: BLE001 - tenta de novo
                pass

    @property
    def supports_calibration(self) -> bool:
        return True

    def calibrate(self) -> None:
        """Envia o comando de calibração (zera tilt e pan) ao escravo Modbus.

        Bloqueante — deve ser chamado fora da thread da UI. Levanta
        `RuntimeError`/`IOError` se não estiver conectado ou se a escrita falhar.
        """
        with self._client_lock:
            if self._client is None:
                raise RuntimeError("Não conectado ao dispositivo.")
            result = self._client.write_coil(address=CALIBRATE_COIL, value=True, device_id=self._slave_id)
            if result.isError():
                raise IOError(str(result))

    @property
    def supports_peak_reset(self) -> bool:
        return self._caps.peaks

    def reset_peaks(self) -> None:
        """Manda o escravo esquecer os extremos dos dois eixos.

        Não há corrida com a leitura: a escrita e o poll compartilham
        `_client_lock`, então nenhuma resposta com os extremos antigos pode
        estar em trânsito quando esta função retorna.

        Silenciosamente no-op se o firmware não conhecer a coil — nesse caso
        os extremos são do app, e zerá-los ali já resolveu.
        """
        if not self._caps.peaks:
            return
        with self._client_lock:
            if self._client is None:
                raise RuntimeError("Não conectado ao dispositivo.")
            result = self._client.write_coil(
                address=RESET_PEAKS_COIL, value=True, device_id=self._slave_id
            )
            if result.isError():
                if not _is_slave_exception(result):
                    raise IOError(str(result))
                self._caps.peaks = False

    @property
    def supports_vibration_capture(self) -> bool:
        return True

    def start_vibration_capture(self, duration_s: float, rate_hz: float, on_progress, on_done) -> None:
        if self._vibration_thread is not None:
            return
        self._vibration_stop_event.clear()
        self._vibration_thread = threading.Thread(
            target=self._run_vibration_capture,
            args=(duration_s, rate_hz, on_progress, on_done),
            daemon=True,
        )
        self._vibration_thread.start()

    def stop_vibration_capture(self) -> None:
        self._vibration_stop_event.set()
        if self._vibration_thread is not None:
            self._vibration_thread.join(timeout=2.0)
            self._vibration_thread = None

    def _transact(self, operation):
        """Executa `operation(client)` com o cliente CORRENTE, sob o lock, com
        até VIBRATION_TRANSACTION_ATTEMPTS tentativas.

        Numa transferência de milhares de amostras a 9600 baud, uma única
        resposta perdida derrubava a captura inteira; e o cliente guardado no
        início podia já ter sido fechado pela reconexão da leitura contínua.
        `operation` devolve o resultado ou levanta IOError; exceções do
        escravo (endereço inexistente) não são repetidas.
        """
        last_error: Exception | None = None
        for attempt in range(VIBRATION_TRANSACTION_ATTEMPTS):
            if self._vibration_stop_event.is_set():
                raise _CaptureCancelled()
            with self._client_lock:
                client = self._client
                if client is not None:
                    try:
                        return operation(client)
                    except _SlaveRejected:
                        raise
                    except Exception as exc:  # noqa: BLE001 - tenta de novo
                        last_error = exc
                else:
                    last_error = RuntimeError("Não conectado ao dispositivo.")
            self._vibration_stop_event.wait(VIBRATION_TRANSACTION_RETRY_DELAY_S)
        raise IOError(f"Comunicação com o ESP32 falhou durante a captura de vibração: {last_error}")

    def _run_vibration_capture(self, duration_s: float, rate_hz: float, on_progress, on_done) -> None:
        try:
            def configure(client):
                for result in (
                    client.write_register(address=VIBRATION_DURATION_REG, value=int(duration_s), device_id=self._slave_id),
                    client.write_register(address=VIBRATION_RATE_REG, value=int(rate_hz), device_id=self._slave_id),
                ):
                    if result.isError():
                        raise IOError(str(result))

            def start(client):
                result = client.write_coil(address=VIBRATION_START_COIL, value=True, device_id=self._slave_id)
                if result.isError():
                    raise IOError(str(result))

            self._transact(configure)
            self._transact(start)

            def read_status(client):
                result = client.read_input_registers(address=VIBRATION_STATUS_REG, count=3, device_id=self._slave_id)
                if result.isError():
                    raise IOError(str(result))
                return result.registers

            sample_count = 0
            while True:
                if self._vibration_stop_event.is_set():
                    on_done(None, "Captura cancelada pelo usuário.")
                    return
                status, progress, sample_count = self._transact(read_status)
                if status == 2:
                    break
                if status == 3:
                    raise IOError("Firmware reportou erro durante a captura de vibração.")
                if status == 0:
                    # Logo após o comando de início o status já é "capturando";
                    # "ocioso" aqui só acontece se o ESP32 reiniciou no meio.
                    raise IOError("O ESP32 reiniciou durante a captura de vibração — repita a captura.")
                on_progress(float(progress))
                self._vibration_stop_event.wait(VIBRATION_STATUS_POLL_INTERVAL_S)

            on_progress(100.0)
            capture_finished_at = time.time()
            t_start = capture_finished_at - sample_count / rate_hz

            angles: list[float] = []
            pan_rates: list[float] = []
            pan_supported = True
            index = 0
            while index < sample_count:
                block_size = min(VIBRATION_BLOCK_SIZE, sample_count - index)

                def read_block(client, index=index, block_size=block_size):
                    # Cursor e bloco na MESMA transação sob o lock: entre os
                    # dois, ninguém mais mexe no cursor do firmware.
                    cursor_result = client.write_register(
                        address=VIBRATION_CURSOR_REG, value=index, device_id=self._slave_id
                    )
                    if cursor_result.isError():
                        raise IOError(str(cursor_result))
                    block_result = client.read_input_registers(
                        address=VIBRATION_BLOCK_START_REG, count=block_size, device_id=self._slave_id
                    )
                    if block_result.isError() or len(block_result.registers) != block_size:
                        raise IOError(str(block_result))
                    return block_result.registers

                def read_pan_block(client, index=index, block_size=block_size):
                    cursor_result = client.write_register(
                        address=VIBRATION_CURSOR_REG, value=index, device_id=self._slave_id
                    )
                    if cursor_result.isError():
                        raise IOError(str(cursor_result))
                    pan_result = client.read_input_registers(
                        address=VIBRATION_PAN_BLOCK_START_REG, count=block_size, device_id=self._slave_id
                    )
                    if pan_result.isError():
                        # Só desiste do eixo de pan se o escravo rejeitou o
                        # endereço (firmware anterior à v1.3.0); falha de
                        # transporte é repetida por _transact.
                        if _is_slave_exception(pan_result):
                            raise _SlaveRejected()
                        raise IOError(str(pan_result))
                    if len(pan_result.registers) != block_size:
                        raise IOError("bloco de pan incompleto")
                    return pan_result.registers

                angles.extend(_to_signed16(raw) / ANGLE_SCALE for raw in self._transact(read_block))
                if pan_supported:
                    try:
                        pan_rates.extend(_to_signed16(raw) / PAN_RATE_SCALE for raw in self._transact(read_pan_block))
                    except _SlaveRejected:
                        pan_supported = False
                        pan_rates = []
                index += block_size
                # A transferência leva quase tanto tempo quanto a captura em
                # taxas altas (a 9600 bauds, 32 amostras por transação): sem
                # reportar progresso aqui, a barra ficava cravada em 100% e
                # parecia travamento.
                on_progress(100.0 * index / sample_count, "Transferindo amostras do ESP32...")

            on_done(
                build_vibration_readings(
                    angles, pan_rates if pan_supported else None, rate_hz, t_start
                ),
                None,
            )
        except _CaptureCancelled:
            on_done(None, "Captura cancelada pelo usuário.")
        except Exception as exc:  # noqa: BLE001
            on_done(None, str(exc))
        finally:
            self._vibration_thread = None

    def start(self, on_reading: ReadingCallback, on_error: ErrorCallback | None = None) -> None:
        if self._thread is not None:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, args=(on_reading, on_error), daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _open_client(self):
        """Abre a porta e devolve um cliente conectado, ou `None` se não
        conseguiu (ou se pediram parada no meio das tentativas)."""
        _require_pymodbus()
        client = _make_client(self._port, self._baudrate, self._timeout)
        for attempt in range(CONNECT_RETRY_ATTEMPTS):
            if self._stop_event.is_set():
                client.close()
                return None
            if client.connect():
                # Espera o ESP32 responder (ver BOARD_READY_TIMEOUT_S). Se
                # não responder, segue assim mesmo: o laço de leitura trata
                # as falhas e reconecta.
                if _wait_until_ready(client, self._slave_id):
                    self._read_firmware_version(client)
                return client
            if attempt < CONNECT_RETRY_ATTEMPTS - 1:
                self._stop_event.wait(CONNECT_RETRY_DELAY_S)
        client.close()
        return None

    def _run(self, on_reading: ReadingCallback, on_error: ErrorCallback | None) -> None:
        def report(message: str) -> None:
            if on_error and not self._stop_event.is_set():
                on_error(message)

        client = None
        try:
            client = self._open_client()
            if client is None:
                report(f"Não foi possível abrir a porta serial {self._port}.")
                return

            with self._client_lock:
                self._client = client

            consecutive_failures = 0
            reconnects = 0
            while not self._stop_event.is_set():
                try:
                    with self._client_lock:
                        sample = _read_axes(client, self._slave_id, self._caps)
                    if consecutive_failures or reconnects:
                        report("Comunicação com o ESP32 restabelecida.")
                    consecutive_failures = 0
                    reconnects = 0
                    on_reading(AngleReading(timestamp=time.time(), **sample._asdict()))
                except Exception as exc:  # noqa: BLE001 - reporta e tenta se recuperar
                    consecutive_failures += 1
                    # Só reporta a primeira falha da sequência: sem isso, a
                    # mesma mensagem era reescrita na barra de status quatro
                    # vezes por segundo enquanto o problema durasse.
                    if consecutive_failures == 1:
                        report(f"Erro de leitura Modbus: {exc}")
                    if consecutive_failures >= READ_FAILURES_BEFORE_RECONNECT:
                        reconnects += 1
                        if reconnects > RECONNECT_ATTEMPTS:
                            report(
                                f"O ESP32 parou de responder na porta {self._port} e não voltou "
                                f"após {RECONNECT_ATTEMPTS} reconexões — verifique o cabo USB e a "
                                f"alimentação, e use \"Iniciar\" para tentar de novo."
                            )
                            return
                        report(f"Reconectando à porta {self._port} ({reconnects}/{RECONNECT_ATTEMPTS})...")
                        with self._client_lock:
                            self._client = None
                        client.close()
                        client = self._open_client()
                        if client is None:
                            report(
                                f"Não foi possível reabrir a porta serial {self._port} — "
                                f"verifique o cabo USB e se o ESP32 está ligado."
                            )
                            return
                        with self._client_lock:
                            self._client = client
                        consecutive_failures = 0
                        continue
                self._stop_event.wait(self._poll_interval)
        finally:
            with self._client_lock:
                self._client = None
            if client is not None:
                client.close()
            # Sem isto, uma falha de conexão deixava `_thread` apontando para
            # a thread já encerrada, e `start()` — que retorna cedo quando
            # `_thread` não é None — virava um no-op silencioso: clicar em
            # "Iniciar" de novo não fazia absolutamente nada.
            if self._thread is threading.current_thread():
                self._thread = None
