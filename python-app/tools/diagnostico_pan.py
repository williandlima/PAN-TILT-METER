"""Diagnóstico do eixo de pan em campo, via Bluetooth (firmware 1.6.3+).

Lê a characteristic de diagnóstico do firmware (CHAR_PAN_DIAGNOSTICS_UUID em
firmware/src/Config.h) duas vezes por segundo, mostra o estado interno do
PanSensor na tela e grava tudo em um CSV para análise.

Uso (na pasta python-app, com o app principal FECHADO — só um cliente BLE
por vez):
    python tools/diagnostico_pan.py                 # procura "Inclinometro-ESP32"
    python tools/diagnostico_pan.py AA:BB:CC:DD:EE:FF
    python tools/diagnostico_pan.py --segundos 120

Durante a gravação, tecle Ctrl+C para encerrar. Tecle Enter a qualquer
momento para mandar o comando Calibrar ao firmware (marcado no CSV).
"""

import argparse
import asyncio
import csv
import math
import struct
import sys
import threading
import time
from datetime import datetime

SERVICE_NAME = "Inclinometro-ESP32"
DIAG_UUID = "6e6e000d-3c17-4a2e-8f4b-1a2b3c4d5e6f"
PAN_UUID = "6e6e0008-3c17-4a2e-8f4b-1a2b3c4d5e6f"
CALIBRATE_UUID = "6e6e0003-3c17-4a2e-8f4b-1a2b3c4d5e6f"
VERSION_UUID = "6e6e0007-3c17-4a2e-8f4b-1a2b3c4d5e6f"

FIELDS = (
    "pan_interno", "offset", "bias_gy", "bias_gz", "media_gy", "media_gz",
    "gy", "gz", "tilt", "amostras", "falhas_i2c", "janelas_fora_bias", "bias_pronto", "espurias",
    "recuperacoes_mpu", "bias_gx", "gx", "vertical_x", "vertical_y", "vertical_z",
)
_FORMAT = "<9fIIHBII5f"  # 75 bytes (1.6.17+; os 55 primeiros até a 1.6.16), ver CHAR_PAN_DIAGNOSTICS_UUID


def parse_diagnostics(raw: bytes) -> dict:
    raw = bytes(raw)
    if len(raw) < struct.calcsize(_FORMAT):  # firmware antigo: sem os campos do fim
        raw = raw + b"\x00" * (struct.calcsize(_FORMAT) - len(raw))
    d = dict(zip(FIELDS, struct.unpack(_FORMAT, raw[: struct.calcsize(_FORMAT)])))
    # Montagem: ângulo do eixo X do chip fora da horizontal (0 = X horizontal,
    # como previsto). Só a partir da 1.6.17 (antes, vertical zerada).
    vx, vy, vz = d["vertical_x"], d["vertical_y"], d["vertical_z"]
    n = math.sqrt(vx * vx + vy * vy + vz * vz)
    d["eixo_x_fora_graus"] = math.degrees(math.asin(max(-1.0, min(1.0, vx / n)))) if n > 0 else float("nan")
    return d


async def _find_address() -> str:
    from bleak import BleakScanner

    print(f"Procurando '{SERVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(SERVICE_NAME, timeout=10.0)
    if device is None:
        sys.exit("Dispositivo não encontrado. Informe o endereço MAC como argumento.")
    return device.address


async def run(address: str | None, seconds: float | None) -> None:
    from bleak import BleakClient

    address = address or await _find_address()
    calibrate_requests: list[bool] = []

    def wait_enter():
        for _ in sys.stdin:
            calibrate_requests.append(True)

    threading.Thread(target=wait_enter, daemon=True).start()

    filename = f"diagnostico_pan_{datetime.now():%Y%m%d_%H%M%S}.csv"
    async with BleakClient(address, winrt={"use_cached_services": False}) as client:
        version = await client.read_gatt_char(VERSION_UUID)
        code = version[0] | (version[1] << 8)
        print(f"Conectado a {address}, firmware {code // 10000}.{(code // 100) % 100}.{code % 100}")
        if code < 10603:
            sys.exit("Este diagnóstico exige firmware 1.6.3 ou mais novo.")
        print(f"Gravando em {filename}. Enter = Calibrar, Ctrl+C = sair.\n")
        print(f"{'t(s)':>6} {'pan':>8} {'interno':>10} {'bias gy':>8} {'bias gz':>8} "
              f"{'méd gy':>8} {'méd gz':>8} {'tilt':>7} {'X fora':>7} {'amostr/s':>8} {'falhas':>6} {'espúr':>6} {'reset':>5} {'fora':>4} {'pronto':>6}")

        with open(filename, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f, delimiter=";")
            writer.writerow(("t_s", "pan_reportado", *FIELDS, "eixo_x_fora_graus", "evento"))
            start = time.monotonic()
            last_samples, last_t = None, None
            while seconds is None or time.monotonic() - start < seconds:
                event = ""
                if calibrate_requests:
                    calibrate_requests.clear()
                    await client.write_gatt_char(CALIBRATE_UUID, b"\x01", response=True)
                    event = "calibrar"
                    print(">>> Calibrar enviado")
                d = parse_diagnostics(await client.read_gatt_char(DIAG_UUID))
                pan_raw = await client.read_gatt_char(PAN_UUID)
                pan = struct.unpack("<h", bytes(pan_raw[:2]))[0] / 100.0
                t = time.monotonic() - start
                rate = 0.0
                if last_samples is not None and t > last_t:
                    rate = (d["amostras"] - last_samples) / (t - last_t)
                last_samples, last_t = d["amostras"], t
                print(f"{t:6.1f} {pan:8.2f} {d['pan_interno'] - d['offset']:10.2f} "
                      f"{d['bias_gy']:8.3f} {d['bias_gz']:8.3f} {d['media_gy']:8.3f} {d['media_gz']:8.3f} "
                      f"{d['tilt']:7.2f} {d['eixo_x_fora_graus']:7.1f} {rate:8.1f} {d['falhas_i2c']:6d} {d['espurias']:6d} {d['recuperacoes_mpu']:5d} {d['janelas_fora_bias']:4d} "
                      f"{d['bias_pronto']:6d}")
                writer.writerow((f"{t:.2f}", f"{pan:.2f}", *(d[k] for k in FIELDS), f"{d['eixo_x_fora_graus']:.1f}", event))
                f.flush()
                await asyncio.sleep(0.5)
    print(f"\nArquivo salvo: {filename}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("endereco", nargs="?", help="endereço MAC do ESP32 (opcional)")
    parser.add_argument("--segundos", type=float, help="duração da gravação (padrão: até Ctrl+C)")
    args = parser.parse_args()
    try:
        asyncio.run(run(args.endereco, args.segundos))
    except KeyboardInterrupt:
        print("\nEncerrado.")


if __name__ == "__main__":
    main()
