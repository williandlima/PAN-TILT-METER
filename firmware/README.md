# Firmware — Inclinômetro ESP32

**Versão atual: `1.6.30`** (`firmware/src/Config.h`, `FIRMWARE_VERSION`) —
exposta em runtime tanto por Modbus (input register `REG_FIRMWARE_VERSION`)
quanto por BLE (characteristic `CHAR_FIRMWARE_VERSION_UUID`), como inteiro
`major*10000 + minor*100 + patch` (`FIRMWARE_VERSION_CODE`; ex: `1.0.0` →
`10000`). Bump manual em `Config.h` a cada mudança relevante de contrato ou
comportamento — sem isso os apps não têm como saber qual versão do firmware
estão falando.

Firmware do ESP32 que expõe os **dois eixos** medidos pelo MPU6050 — a
inclinação (**tilt**, pelo acelerômetro) e o azimute (**pan**, pelo
giroscópio; ver "Azimute (pan)" abaixo) — tanto por **Modbus RTU via cabo
USB direto** quanto por **Bluetooth LE**, simultaneamente, seguindo os
contratos já assumidos e documentados nos dois apps:

- `python-app/data_source/modbus_source.py` (Modbus RTU / USB)
- `python-app/data_source/ble_source.py` (BLE)
- `android-app/app/.../datasource/BleContract.kt` (BLE)

A comunicação com o PC usa a porta serial USB nativa do ESP32 (a mesma já
usada para gravar o firmware) — **não usa RS485**. Essa decisão foi tomada
porque a distância até o painel de controle (~7m, no mastro do pan-tilt)
excede o alcance confiável de USB simples (~5m), mas um **cabo de extensão
USB ativo** (com amplificador de sinal embutido, vendido pronto para
10-20m) resolve isso sem precisar de RS485/transceptor/terminação de
barramento.

## Estrutura

```
firmware/
  platformio.ini
  src/
    main.cpp             setup()/loop(), instancia e conecta os módulos abaixo
    Config.h              pinagem + constantes de protocolo (Modbus/BLE)
    Mpu6050.h/.cpp         driver I2C mínimo do MPU6050 (sem lib externa)
    AngleSensor.h/.cpp     tilt por atan2 do acelerômetro + calibração (offset)
    PanSensor.h/.cpp       pan por integração do giroscópio + ZUPT
    PeakHold.h/.cpp        extremos (mín/máx) medidos a 100Hz, com teste
                           de persistência contra ruído
    VibrationCapture.h/.cpp  motor de captura em alta taxa, compartilhado
                             pelos dois transportes
    ModbusSlave.h/.cpp    escravo Modbus RTU sobre a porta serial USB (UART0)
    BleServer.h/.cpp      servidor BLE (GATT)
```

## Pinagem (ESP32-DevKitC V4 / WROOM-32)

Hardware confirmado: placa oficial **ESP32-DevKitC V4** (Espressif, módulo
WROOM-32), com chip conversor USB-serial **CP2102N** (Silicon Labs) —
mapeamento definido, refletido em `firmware/src/Config.h`. Como a
comunicação com o PC vai pela porta USB nativa do ESP32, o único hardware
extra é o MPU6050:

| Sinal | Pino ESP32 | Vai para | Observação |
|---|---|---|---|
| SDA | GPIO 21 | SDA do MPU6050 | Padrão I2C do ESP32 |
| SCL | GPIO 22 | SCL do MPU6050 | Padrão I2C do ESP32 |
| — | 3.3V | VCC do MPU6050 | Não usar 5V (linhas I2C ficam em 5V e podem danificar o ESP32) |
| — | GND | GND do MPU6050 | |
| — | GND | AD0 do MPU6050 | Fixa endereço I2C em `0x68` (o que `Mpu6050.h` assume) |

Se o hardware definitivo usar uma variante diferente do ESP32 (S3, C3,
etc.), os pinos podem precisar de ajuste — essas variantes têm GPIOs
restritos diferentes do WROOM-32 clássico assumido aqui.

A fórmula do ângulo em `AngleSensor.cpp` (`atan2(ay, az)`) assume uma
orientação de montagem do MPU6050 já **recomendada, mas ainda não
confirmada fisicamente**: eixo X do sensor alinhado ao eixo mecânico de
giro do pan-tilt, com a rotação de -60° a +60° acontecendo no plano Y-Z (os
dois eixos usados no `atan2`). Essa combinação mantém a sensibilidade da
leitura quase constante em toda a faixa — sem a zona de baixa sensibilidade
que um único eixo teria perto de 90°, o que é essencial para o **Modo
Vibração** detectar variações de frações de grau em torno do zero
calibrado. Ver "Orientação de montagem do sensor" em `docs/pinout.md`
para o detalhamento e o teste de bancada de confirmação.

## Contrato implementado

### Modbus RTU via USB (escravo, ID configurável em `Config.h`, padrão 1)

Trafega pela mesma porta serial que aparece no PC como uma porta COM/tty
normal quando o ESP32 é conectado por USB — o app desktop (`python-app`)
já espera exatamente isso (não faz distinção entre "porta serial real" e
"porta serial via USB do ESP32", já que ambas são portas seriais comuns do
ponto de vista do `pyserial`).

| Registrador | Tipo | Função |
|---|---|---|
| Input reg. 0 | leitura | Ângulo de **tilt** * 100 (int16, com sinal, faixa -60~+60°) |
| Input reg. 1 | leitura | Ângulo de **pan** * 100 (int16, com sinal, faixa `PAN_MIN_DEG`~`PAN_MAX_DEG`) |
| Input reg. 2-3 | leitura | Mín / máx de **tilt** * 100 (int16, medidos no firmware — ver "Extremos" abaixo) |
| Input reg. 4-5 | leitura | Mín / máx de **pan** * 100 (int16) |
| Coil 0 | escrita | `true` → calibra (zera **os dois eixos**, tilt e pan, e os extremos junto) |
| Coil 1 | escrita | `true` → inicia captura de vibração |
| Coil 2 | escrita | `true` → esquece os extremos (sem mexer no zero) |
| Holding reg. 10 | escrita | Duração da captura (s) |
| Holding reg. 11 | escrita | Taxa de amostragem (Hz) |
| Input reg. 20-22 | leitura | Status / progresso (%) / total de amostras da captura |
| Holding reg. 30 | escrita | Cursor (índice inicial do bloco a ler) — vale para os dois eixos |
| Input reg. 31-62 | leitura | Bloco de até 32 amostras de **tilt** (int16, ângulo relativo * 100) |
| Input reg. 70-101 | leitura | Bloco de até 32 amostras de **pan** (int16, **velocidade angular** em °/s * 100) |
| Input reg. 40 | leitura | Versão do firmware (`FIRMWARE_VERSION_CODE`, ver acima) |

### BLE (serviço `6e6e0001-...`)

| Characteristic | Propriedade | Função |
|---|---|---|
| `6e6e0002-...` (tilt) | read/notify | Ângulo de tilt * 100 (int16 LE, com sinal, faixa -60~+60°), a cada ~200ms |
| `6e6e0003-...` (calibrar) | write | Byte `0x01` → calibra **os dois eixos** |
| `6e6e0004-...` (config vibração) | write | 4 bytes LE: duração(s) + taxa(Hz) → inicia captura |
| `6e6e0005-...` (status vibração) | notify | status/progresso/total de amostras |
| `6e6e0006-...` (dados vibração) | notify | Amostras em pacotes (índice + até 8 amostras int16 LE) |
| `6e6e0007-...` (versão firmware) | read | `FIRMWARE_VERSION_CODE` (uint16 LE, ver acima) — valor fixo, sem notify |
| `6e6e0008-...` (pan) | read/notify | Ângulo de pan * 100 (int16 LE, com sinal), na mesma cadência do tilt |
| `6e6e0009-...` (dados vibração pan) | notify | Amostras de pan em pacotes (índice + até 8 int16 LE), em **velocidade angular** °/s * 100 |
| `6e6e000a-...` (retransmissão) | write | 3 bytes: índice inicial (uint16 LE) + eixo (0=tilt, 1=pan) → reenvia as amostras a partir dali |
| `6e6e000b-...` (resetar extremos) | write | Byte `0x01` → esquece os extremos (sem mexer no zero) |
| `6e6e000c-...` (extremos) | read/notify | 8 bytes = 4 int16 LE * 100: tilt mín, tilt máx, pan mín, pan máx. Notifica só quando muda |

O pan entrou numa characteristic própria, e não anexado à de tilt, de
propósito: os apps já instalados esperam exatamente 2 bytes em
`6e6e0002-...`, então estender aquela mensagem os quebraria. Do jeito que
está, a mudança é puramente aditiva — um app que não conhece
`6e6e0008-...` simplesmente a ignora e continua funcionando como antes.

Detalhes byte a byte de cada mensagem estão comentados no topo de
`ModbusSlave.cpp`/`BleServer.cpp` e nos módulos Python/Kotlin equivalentes.

## Build

Projeto [PlatformIO](https://platformio.org/), ambiente `esp32dev`
(framework Arduino, sem bibliotecas externas — só o que já vem com o core
`espressif32`: `Wire`, UART, e a lib BLE clássica `BLEDevice`).

```bash
cd firmware
pio run              # compila
pio run -t upload    # compila e grava no ESP32 conectado por USB
pio device monitor    # abre o monitor serial (115200 baud)
```

**Testes sem hardware**: `sim/run.sh` (na raiz do repositório) compila o
código real de `firmware/src` no computador, contra um MPU6050 e uma lib BLE
simulados, e roda o pan com mudanças de posição da placa e o Modo Vibração
via BLE de ponta a ponta com o código real do app Python (inclusive com
perda de pacotes). Não substitui o teste em bancada — não mede tempo real
de I2C/rádio —, mas pega erros de lógica e de contrato BLE.

O **teste de resistência** (`sim/soak_sim.cpp`, parte do `run.sh`) roda o
firmware inteiro — inclusive o driver I2C real, `Mpu6050.cpp` — contra um
MPU6050 emulado no nível de registrador, por 30 min simulados em cada uma
de 20 sementes (`SOAK_SEEDS=100 sim/run.sh` para mais), com giros e
mudanças de tilt aleatórios (inclusive além do curso), uma calibração no
meio, bias de fábrica de até ±20°/s com deriva térmica de ±3°/s, vibração,
e falhas de hardware injetadas: NACK e leitura curta, quadros de lixo e
0xFF, picos isolados no giro, barramento travado, reset do sensor (volta
em SLEEP lendo zeros) e loop parado. Ao fim de cada período parado a
leitura é comparada com a verdade física: tolerância de 1,0° no pan e
0,3° no tilt. Com o `PanSensor` da 1.6.4 o pior erro é de 85° (trava no
limite); na 1.6.5, 0,57° (100 sementes: mediana 0,29°, pior 0,96°).
`SOAK_FAULTS` (máscara de bits, ver o código) liga cada tipo de falha
isoladamente, e `SOAK_VERBOSE`/`SOAK_TRACE` ajudam a investigar.

**Diagnóstico do pan em campo** (1.6.3+): a characteristic read-only
`CHAR_PAN_DIAGNOSTICS_UUID` expõe o estado interno do `PanSensor` (integrador
sem clamp, offset, bias por eixo, médias da última janela, amostra crua,
tilt, contadores de amostras e de falhas de I2C). `python-app/tools/
diagnostico_pan.py` lê isso via Bluetooth duas vezes por segundo, mostra na
tela e grava um CSV — Enter envia Calibrar, marcado no arquivo. Os apps não
usam essa characteristic.

**Não foi possível compilar neste ambiente de desenvolvimento**: o
`pio run` precisa baixar o toolchain/plataforma `espressif32` da internet
na primeira vez, e o acesso de rede deste sandbox bloqueia esse download
(mesma limitação de rede já encontrada antes neste projeto, ex: para
baixar a logo da Avibras Aeroco). O código foi revisado manualmente com
cuidado (tipos, includes, formato de cada mensagem confrontado byte a byte
com os módulos Python/Kotlin), mas **precisa ser compilado e testado em
hardware real** antes de considerar o firmware pronto — nem a lógica de
sensor, nem os protocolos Modbus RTU (via USB)/BLE foram validados contra
hardware físico.

## Estabilidade da leitura (v1.1.0)

Nos primeiros testes com hardware real a leitura não parava de variar nos
centésimos de grau. Eram duas causas, as duas no firmware:

1. **O filtro passa-baixa interno do MPU6050 nunca era configurado.** O chip
   liga com `DLPF_CFG=0`, ou seja, 260Hz de banda passante no acelerômetro —
   praticamente sem filtro. Todo o ruído mecânico/térmico de alta frequência
   entrava em cada amostra. Isso ainda causava **aliasing** no Modo Vibração:
   amostrando a 50Hz (Nyquist 25Hz) um sinal de 260Hz de banda, o ruído acima
   de 25Hz era rebatido para dentro da faixa medida e sujava a FFT. Agora
   `Mpu6050::begin()` configura `DLPF_CFG=4` (21Hz), que corta esse ruído no
   hardware e serve de anti-aliasing, ainda deixando passar com folga as
   frequências de interesse do ensaio (1-5Hz).
2. **A leitura contínua não tinha filtragem nenhuma.** Cada leitura Modbus /
   notify BLE disparava uma única amostra instantânea direto no `atan2`.
   Agora `AngleSensor::update()` (chamado no `loop()`) amostra o sensor a
   100Hz e mantém uma média móvel exponencial com constante de tempo de 0.5s
   (`ANGLE_FILTER_TIME_CONSTANT_S`), e é esse valor filtrado que
   `readAngleDeg()` devolve.

**O Modo Vibração não passa por esse filtro, de propósito** — é justamente
ali que a sensibilidade a frações de grau é o objetivo. A captura continua
usando `readRelativeAngleDeg()`, que faz uma leitura instantânea a cada
amostra. Os dois caminhos estão separados em `AngleSensor.h`.

A calibração também passou a usar o valor filtrado em vez de uma amostra
isolada, ficando repetível mesmo com o sensor sob vibração.

Do lado dos apps, a exibição do valor em tempo real é arredondada para
degraus de 0,25° com histerese (evita alternar entre dois degraus quando o
valor fica na fronteira). Isso é só apresentação: histórico, mín/máx e
relatórios continuam usando o ângulo bruto.

### Fator de escala (`TILT_SCALE_CORRECTION_NEG`/`_POS`, v1.6.8/1.6.10/1.6.11/1.6.12/1.6.13)

Resolvido pelo `atan2` entre os dois eixos do acelerômetro (sem singularidade
nem necessidade de bias, diferente do giroscópio do pan), o tilt não deveria
sofrer do mesmo erro de escala do pan — mas um erro de bancada mostrou que
sofre: um descasamento de sensibilidade entre os eixos Y/Z do acelerômetro
usados no `atan2`, com a mesma assinatura (erro proporcional ao deslocamento
em relação à calibração, não um offset fixo).

**Calibrado na 1.6.8** com dados de bancada reais: inclinômetro de referência
Mitutoyo PRO3600 (nº F4020023), comparado com a leitura do PAN-TILT METER em
24 pontos entre −8,6° e +7,7° de inclinação (esses mesmos 24 pontos foram
atribuídos por engano ao azimute na 1.6.7 — ver `PAN_SCALE_CORRECTION`
acima). Antes da correção o desvio-padrão do erro era 0,255°; um ajuste de
escala pura (sem deslocamento) reduziu para 0,073° (máximo 0,20°) — usando
uma única constante (`1,051`) para os dois lados.

**Refinado na 1.6.10**: separando os mesmos 24 pontos por sinal, o fator
medido não é igual dos dois lados — 0,9593 do lado negativo, 0,9408 do
positivo. Uma correção única deixava um viés residual de −0,047°/−0,076° em
cada lado (confirmado depois em campo com pontos novos entre +1° e +2°).
Duas constantes agora, `TILT_SCALE_CORRECTION_NEG` e `_POS`, escolhidas pelo
**sinal do ângulo bruto absoluto** (`rawDeg`, antes de subtrair a
calibração) — não pelo sinal relativo à calibração. Isso importa: a
assimetria é uma característica física do sensor/montagem, fixa no zero
MECÂNICO, então calibrar longe dele (ex.: com o pan-tilt já inclinado) não
pode fazer a leitura escolher o lado errado.

**Lado positivo recalibrado na 1.6.11** com 15 pontos novos de bancada
(0,48° a 21,5°, complementando os 9 originais que iam só até 6,99°): o
fator de `1,063` vinha ficando cada vez mais curto acima de uns 4°,
chegando a −0,30° de erro em 21,5° — sinal de que 8,6° não era faixa
suficiente para extrapolar. Com os 24 pontos positivos juntos, o fator
medido é 0,9291 (era 0,9408 só com os 9 originais); `TILT_SCALE_CORRECTION_POS`
passou para `1,076`, e o resíduo caiu para entre −0,05° e +0,03° na faixa
testada até ali.

**1.6.12 (REVERTIDO na 1.6.13):** uma rodada extra de bancada (0° a 8,03°)
parecia mostrar viés crescente com o fator `1,076`, e o fator foi para
`1,088` a partir disso. O cálculo estava errado: a placa usada nas duas
últimas rodadas de bancada (0,48° a 21,5° e 0° a 8,03°) nunca tinha sido
regravada com a 1.6.10/1.6.11 — continuava em `1.6.8`, com a correção
única de `1,051`. As leituras já vinham com `1,051` aplicado, e o cálculo
as dividia por `1,076` para achar o valor bruto do sensor — dividindo pelo
fator errado, a conversão saía errada, e o ajuste seguinte (`1,088`)
herdou esse erro.

**Recalibrado corretamente na 1.6.13**, com as mesmas três rodadas (9 + 15
+ 15 = 39 pontos positivos, 0° a 21,5°), convertendo as duas últimas para
bruto com o fator que a placa de fato tinha (`1,051`): o fator medido é
`0,9383`. `TILT_SCALE_CORRECTION_POS` volta para **1,066** — desvio-padrão
do resíduo cai para `0,062°` e viés médio para `−0,008°` (o melhor
resultado desde o início; `1,088` dava desvio `0,123°`). A lição prática:
ao reaproveitar uma leitura já corrigida pelo firmware para recalibrar, é
preciso confirmar a versão REALMENTE gravada na placa (characteristic/
registrador de versão — o botão "Testar conexão" dos apps mostra isso),
não assumir pela palavra de quem fez o teste.

O lado negativo (`TILT_SCALE_CORRECTION_NEG = 1,042`) segue com só os 9
pontos originais (até −8,6°), colhidos sem nenhuma correção de firmware —
sujeito ao mesmo risco de ficar curto fora dessa faixa; mais pontos desse
lado, numa faixa maior, teriam o mesmo benefício. Refazer toda esta
calibração se o MPU6050 físico desta unidade for substituído.

### Tilt: erro maior no lado negativo — análise dos métodos (v1.6.20)

Relato de campo: o erro do tilt é maior nas posições negativas do que nas
positivas. A cadeia de cálculo é `atan2(ay, az)` → fator de escala por lado
(`TILT_SCALE_CORRECTION_NEG` 1,042 / `_POS` 1,066, escolhido pelo sinal do
ângulo bruto) → subtração do zero da calibração. Três problemas de método:

1. **O modelo está errado na forma.** Um fator por lado é uma reta por lado,
   com uma quebra no zero do bruto. O erro real de `atan2(ay, az)` vem de
   imperfeições do acelerômetro aplicadas a `ay` e `az` — ganho relativo Y/Z
   (ou desalinhamento da placa, que tem o mesmo efeito) e offsets de zero-g.
   O ganho dá um erro ímpar (igual dos dois lados); o **offset do eixo Y dá
   um erro PAR, ≈ −b_y·θ²/2**, de mesmo sinal nos dois lados. Somado ao
   ímpar, ele aumenta o erro de um lado e diminui do outro — e cresce com o
   quadrado do ângulo, então nenhum fator linear acompanha.
2. **Os dois lados foram calibrados em faixas diferentes.** O positivo tem
   39 pontos até +21,5°; o negativo, 9 pontos até −8,6°. O próprio histórico
   do lado positivo mostra o efeito do item 1: ajustado até 7°, o fator
   (1,063) errava −0,30° em 21,5°, e precisou ir para 1,066 com a faixa
   maior. No lado negativo esse ajuste nunca foi feito: abaixo de −8,6° o
   fator é extrapolação.
3. **A quebra fica no zero do bruto, não no zero do pan-tilt.** Se o bruto
   no zero da calibração não for 0° (offset ou montagem), as posições
   negativas pequenas usam o fator do lado positivo.

**Modelo numérico** (`ay = k·sen θ + b_y`, `az = cos θ + b_z`, ruído real):
os fatores medidos em bancada (0,9593 / 0,9383) são reproduzidos com
`k ≈ 0,97` e `b_y ≈ +136 mg`. Com eles, o firmware atual erra:

| Tilt | −60° | −45° | −30° | −21,5° | −15° | −8,6° | +8,6° | +21,5° | +30° | +45° | +60° |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Atual (1,042/1,066) | −5,2° | −2,7° | −1,1° | −0,6° | −0,3° | −0,2° | +0,05° | −0,08° | −0,3° | −0,7° | −1,3° |

— o padrão relatado: pequeno dentro das faixas calibradas, crescendo muito
mais rápido no lado negativo. Comparação de métodos, de −60° a +60°:

| Método | Lado negativo (máx) | Lado positivo (máx) |
|---|---|---|
| Atual (fatores de bancada) | 5,2° | 1,3° |
| Mesmo método, fatores refeitos com pontos de −60 a +60 | 2,1° | 0,3° |
| Polinômio de 3º grau no bruto, pontos de −60 a +60 | 0,09° | 0,05° |
| **Modelo físico `atan2((ay−b_y)/k, az−b_z)` ajustado a pontos** | **0,03°** | **0,04°** |

Uma varredura de tilt sem instrumento de referência (círculo com |a| = 1 g)
acha offsets e desalinhamento sozinha, mas é sensível à tolerância de ganho
do eixo Z (±3% → ~1°): por isso o ajuste usa pontos de referência, e nele só
a razão dos ganhos Y/Z entra (exata).

**O que mudou na 1.6.20:** o tilt passa a ser
`atan2((ay − TILT_ACCEL_OFFSET_Y_G)/TILT_ACCEL_GAIN_Y, az − TILT_ACCEL_OFFSET_Z_G)`,
com padrões (1, 0, 0) — **leitura idêntica à da 1.6.19** até as constantes
serem medidas. Elas saem de `python-app/tools/calibra_tilt.py` (com
`--autoteste` no `sim/run.sh`), e então os fatores NEG/POS voltam a 1,0.

**Procedimento de bancada para fechar a calibração:**
1. Rodar `python tools/diagnostico_pan.py` (grava o ângulo **bruto**, coluna
   `tilt`, que não depende dos fatores nem da versão do firmware — evita o
   erro da 1.6.12).
2. Com o PRO3600, posicionar o tilt em pontos dos **dois lados** e na faixa
   toda (ex.: −60 a +60 a cada 5° ou 10°, incluindo o 0), parando ~5 s em
   cada um.
3. Montar um CSV `referencia;bruto;leitura` (bruto = média dos ~5 s) e rodar
   `python tools/calibra_tilt.py pontos.csv`: ele mostra o erro do método
   atual e do ajustado por lado e imprime as constantes para `Config.h`.

O mesmo offset afeta a vertical usada pelo pan (`u = a/|a|`), mas lá entra
como cos do desvio (~0,9% com 136 mg), confundido hoje dentro de
`PAN_SCALE_CORRECTION`; aplicá-lo ao pan fica para depois de medido.

### Pan: reavaliação do método e comparação com a prática de especialistas (v1.6.21)

**Orçamento de erro** de um giro de 90°, com as tolerâncias do MPU6050
(datasheet) e a placa montada torta (eixo X a 44°, ver cenário U):

| Fonte | Erro num giro de 90° | Tratado? |
|---|---|---|
| Escala única (1,030) de **2 pontos de 8°/9°** | ±1,1° por 0,1° de incerteza da referência | **não** (dados insuficientes) |
| Escala que **muda com o tilt** (nᵀ·M·n: ±3% por eixo, ±2% cruzada) | 0,4° (30°), 0,8° (60°), até ~2° (95%) | **não** |
| Assimetria de escala horário × anti-horário (MEMS 0,5–1%) | ±0,7° | **não** |
| Bias (zero do giro) e seu passeio | ~0,3° (cenários P/T) | sim (ZUPT) |
| Desnível da viatura / balanço do mastro | até 3–8° | sim (1.6.19) |
| Ruído branco (ARW) | ~0,01° por giro | desprezível |

Resolvido o bias, **o erro relevante de um giro é a escala** — e é
proporcional ao giro, por isso aparece justamente nos movimentos grandes.

**Prática de quem trabalha com giroscópio × firmware:**

| Aspecto | Prática de especialistas (IMU/AHRS) | Firmware | Situação |
|---|---|---|---|
| Integração | quaternion / vetor de rotação (efeito *coning*) | projeção da taxa no eixo de pan, trapézio a 100 Hz | equivalente para um pan-tilt (rotações em torno de dois eixos fixos) |
| Bias online | Kalman com estado de bias + ZUPT/ZARU | filtro alfa-beta (Kalman de ganho fixo) + ZUPT por janela | equivalente na essência |
| Yaw sem magnetômetro | não observável: depende de calibração e de bias | idem | limitação física, igual |
| **Escala e desalinhamento** | **matriz 3x3 calibrada** em mesa giratória, vários eixos/posições (IEEE 1293) | **um número, 2 pontos** | **lacuna principal** → 1.6.21 |
| Bias × temperatura | compensação pela temperatura do chip | temperatura lida no burst e descartada; inclinação temporal ≤30 s | recomendação |
| Vertical | filtro complementar (Mahony/Madgwick) rejeita aceleração linear | acelerômetro direto (mediana) | efeito de 2ª ordem no pan; recomendação |
| Caracterização de ruído | variância de Allan define limiares | limiares empíricos (0,1 °/s) | 1.6.21: ferramenta |
| Referência absoluta | magnetômetro / GNSS / encoder | — | o encoder da PTU-D300E é a referência natural |

Um filtro de Kalman completo no lugar do ZUPT **não** resolveria o erro
relevante: a escala não é observável sem uma referência externa de giro.
O caminho dos especialistas para ela é calibração, e é o que esta versão
habilita:

- **Firmware 1.6.21:** escala em função do tilt e do sentido,
  `k(t) = A + B·cos 2t + C·sin 2t` (exato para nᵀ·M·n no plano do tilt; t =
  tilt bruto) vezes `(1 ± PAN_SCALE_ASYM)`. Padrão `B = C = ASYM = 0` →
  **leitura idêntica à 1.6.20**.
- **`python-app/tools/calibra_pan.py`:** ajusta A, B, C e a assimetria a
  giros de referência e imprime as constantes. Validado no firmware real
  (simulador, giroscópio com tolerâncias sorteadas): pior erro de giro
  **1,88° → 0,18°**.
- **`python-app/tools/allan_pan.py`:** variância de Allan de uma captura do
  Modo Vibração com o pan parado (ARW, instabilidade de bias e folga do
  limiar do ZUPT para esta unidade).

**Procedimento de bancada (escala):** com a PTU-D300E comandada pelo
Docklight (posições absolutas `PP`, encoder de milésimos de grau) ou uma
mesa giratória, fazer giros de ±90° e ±45° nos **dois sentidos** em tilts
de −30°, 0°, 30° e 60°, anotando o tilt bruto (coluna `tilt` do
`diagnostico_pan.py`), o giro real e o giro lido (pan antes/depois), e rodar
`python tools/calibra_pan.py giros.csv` (modelo: `modelo_giros_pan.csv`).

**Próximos passos recomendados**, em ordem de ganho: (1) a calibração de
escala acima; (2) compensação térmica do bias com a temperatura do chip
(movimentos longos e aquecimento); (3) vertical por filtro complementar,
se houver aceleração linear forte no topo do mastro; (4) se o que importa
é a posição do pan da PTU (e não a torção do mastro), ler o próprio encoder
da PTU pelo RS-485.

### Zero do tilt salvo, watchdog e compilação automática (v1.6.22)

- **Zero do tilt na memória permanente (NVS).** Até a 1.6.21 o zero da
  calibração ficava só na RAM: ao desligar o ESP32 ele se perdia, sem aviso,
  e o tilt voltava a ser lido em relação ao zero do sensor. Agora o zero é
  gravado quando muda (só ao calibrar — desgaste da flash desprezível) e
  restaurado ao ligar. O **pan** continua recomeçando em 0° na posição em
  que o ESP32 ligar: ele é integrado do giroscópio e não tem referência
  absoluta.
- **Watchdog do loop principal** (`WATCHDOG_TIMEOUT_S` = 10 s): se o
  firmware travar, o ESP32 reinicia sozinho; os apps reconectam e o zero do
  tilt volta da memória.
- **Compilação no GitHub** (`.github/workflows/firmware.yml`): o firmware é
  compilado com o PlatformIO a cada alteração, e o `firmware.bin` fica como
  artifact.

Testado em `sim/nvs_sim.cpp` (calibra, "desliga e religa" com a NVS em
arquivo, confere o zero e que a flash é gravada uma única vez). **Pendente
em bancada:** calibrar, desligar a alimentação, religar e conferir que o
tilt continua em 0° na mesma posição; o watchdog só é exercitado de verdade
no hardware.

### Tilt: fusão do giroscópio com o acelerômetro (v1.6.23)

**O problema.** O acelerômetro não distingue inclinação de aceleração
linear. No topo do mastro, o balanço da viatura e do próprio mastro (0,5 a
2 Hz) é em boa parte TRANSLAÇÃO: 1 cm de vai-e-vem a 1 Hz já é lido como
±2,3° de "inclinação" que não existe. Como isso fica abaixo do corte de 3 Hz
do caminho de medida, entrava inteiro nos mín/máx e no relatório. (A vibração
do motor, a ~17 Hz, o caminho de medida já rejeitava.)

**O método** é o filtro complementar usado em IMU/AHRS: o giroscópio dá a
variação rápida do ângulo (ele não sente aceleração linear) e o acelerômetro
corrige devagar a deriva do giro (ele tem a referência absoluta da
gravidade), com constante de tempo `TILT_FUSION_TAU_S` = 1 s. Rotação real
passa inteira; aceleração linear na frequência f sobra multiplicada por
~1/(2π·f·τ). O bias do giro parte do ZUPT do pan e é acompanhado pelo termo
integral da fusão (revisto na 1.6.24 — ver a seção seguinte). A tela e os mín/máx usam o ângulo
fundido; o **Modo Vibração não muda** (acelerômetro cru). A mesma correção
por eixo do acelerômetro (`TILT_ACCEL_*`) passa a valer também para a
vertical usada no pan — sem efeito enquanto as constantes forem (1, 0, 0).

**Salvaguardas** — em qualquer uma delas vale só o acelerômetro, como até a
1.6.22: bias do giro ainda não confirmado (primeiros segundos após ligar),
|aceleração| fora de 1 g ± 0,15, giro e acelerômetro divergindo mais de 3°,
buraco de amostragem. `TILT_FUSION_ENABLED = false` desliga tudo.

**Resultado no simulador** (`sim/tilt_sim.cpp`: `AngleSensor` e `PanSensor`
reais, dois sensores lado a lado lendo o mesmo MPU6050 falso — giro com bias
de fábrica, 3% de erro de escala e ruído):

| Cenário | Só acelerômetro (≤1.6.22) | Fundido (1.6.23) |
|---|---|---|
| Translação a 1 Hz, ±1° aparentes, 0° reais — faixa mín/máx | 2,00° | 0,40° |
| Translação a 2 Hz, ±1° aparentes, 0° reais — faixa mín/máx | 1,53° | 0,21° |
| Vibração a 17,6 Hz, ±2° aparentes — faixa mín/máx | 0,15° | 0,08° |
| Parado 10 min — maior diferença entre os dois | — | 0,015° |
| Rajada REAL de 2° em 0,5 s — máximo registrado | 1,84° | 1,72° |
| Balanço real ±1° a 1 Hz + vibração (real 2,00°) | 1,83° | 1,80° |
| Motor 0→30° a 15 °/s, giro +3% — passa do ponto final | — | 0,23° |
| Leituras espúrias do giro (500 °/s) — faixa mín/máx | — | 0,06° |

**O preço, para ficar claro:** (1) num movimento rápido o ângulo fundido
herda o erro de escala do giro por ~1 s: a rajada real de 2° sai 0,1° menor
e um movimento do motor a 15 °/s passa ~0,2° do ponto antes de assentar;
(2) depois de um movimento a tela assenta um pouco mais devagar nos últimos
centésimos de grau (o sinal fundido é mais limpo e o filtro da tela fecha
mais cedo). Parado, a leitura é a do acelerômetro — a exatidão estática e a
calibração de bancada (`calibra_tilt.py`) não mudam.

**Pendente em bancada:** só simulação até aqui. Conferir (a) parado: mesma
leitura da 1.6.22; (b) movimento do tilt pela PTU: sem ultrapassagem
visível; (c) no mastro, com a viatura ligada: mín/máx mais estreitos que
antes. Se algo piorar, `TILT_FUSION_ENABLED = false`.

### Pan: passos curtos e rápidos com volta ao zero (v1.6.30)

Bancada com a 1.6.29 (18/11/2026): a melhor leitura de pan até aqui, com
poucos erros. O maior apareceu em movimentos **curtos (5°, depois 10°) e
relativamente rápidos, voltando ao zero**. O método (giro integrado, ZUPT
por janela de 1 s, filtro alfa-beta do bias, proteções da 1.6.29) e a escala
(1,027) não mudam; a correção é dentro do ZUPT.

- **Causa.** A janela de 1 s é julgada pela média. Um passo curto termina
  perto do começo da janela seguinte e deixa nela só a frenagem final: cerca
  de 0,1° em 0,1 s, ou seja, média de 0,1°/s, abaixo do limiar de "parado".
  A janela era dada como parada: os ~0,1° eram **cancelados** e o bias
  **aprendia 30% da frenagem** como se fosse deriva (0,03°/s), depois
  extrapolada pela inclinação do filtro. O mesmo valia para a partida do
  passo seguinte no fim de uma janela. Com paradas curtas entre os passos não
  sobra janela parada limpa para corrigir, e o erro soma a cada passo, em
  qualquer sentido. Agravante: a aceleração do próprio giro entrava na medida
  de vibração (diferença entre amostras sucessivas) e subia o limiar
  justamente dessa janela.
- **Correção.** A janela é dividida em blocos de 250 ms
  (`PAN_ZUPT_SUB_*` em `Config.h`). Ela passa a ser movimento (giro mantido,
  bias não aprende) quando o bloco de maior giro passa de 0,2°/s (ou do
  resíduo de vibração), o giro dele ficou na janela, concentra o giro da
  janela e é coerente (todas as amostras no mesmo sentido). Cada condição
  tem um motivo testado: vibração vai e volta no bloco (coerência; sem ela o
  cenário O de vibração forte errava 13° a 20°); bias errado constante se
  espalha igual pelos blocos (concentração); a vibração medida para esse
  teste usa a 2ª diferença no eixo de pan, que não enxerga a rampa de
  aceleração; e janela com buraco de amostragem fica de fora, porque o buraco
  é integrado de uma vez e parece giro concentrado (sem isso, o custo das
  falhas no soak subia de +0,10° para +0,46°). O limiar principal da janela
  ficou igual ao da 1.6.29.
- **Simulação (PanSensor real, com o filtro de 21 Hz do MPU no modelo).**
  12 passos de 5°/10° com volta ao zero, paradas de 1 s, 100°/s²: 1.6.29
  erra até 0,75°, 1.6.30 até 0,12°. 60 sequências aleatórias de 30 passos de
  2-12° (10-80°/s, 50-400°/s², paradas de 0,3-4 s): volta ao zero com erro
  médio de 0,32° e máximo de 1,79° na 1.6.29; 0,09° e 0,31° na 1.6.30. Teste
  permanente em `sim/vaivem_sim.cpp` (a 1.6.29 falha nele). pan_sim, soak de
  40 sementes e as demais simulações seguem iguais ou melhores (soak sem
  falhas: média do pior erro por semente 0,566° → 0,554°).
- **O que sobra.** Cada passo rápido perde ~1% do giro no pico de velocidade
  (a mediana de 5 amostras corta o bico do perfil); isso se desfaz na volta
  pelo mesmo caminho e não acumula. Não foi mexido.

**Pendente em bancada:** repetir a sequência 5° → 10° → 0 rápida, com
paradas curtas, várias vezes nos dois sentidos, e comparar com a 1.6.29.

### Pan: o limite de ±90° cortava a leitura; escala de volta a 1,027 (v1.6.28 e v1.6.29)

Bancada com a 1.6.27, calibrado em 0, giros de 30°: 0 → 30 → 60 → 90 → 0 →
−30 → −60 → −90 lendo 30,3 / 60,9 / **90,0** / −1,3 / −31,9 / −62,3 /
**−90,0**.

- **Corte em ±90°.** Com o pan lendo ~1,5% a mais, a ida a +90° daria 91,4°;
  o limite da faixa (`PAN_MAX_DEG` = 90, que também prende o integrador)
  segurou em 90,0 e jogou fora 1,4°: a volta ao zero leu −1,3° em vez de
  ~+0,1°. O limite vai para **±320°**: deixa de ser "faixa mecânica" e fica só
  como trava contra integrador disparado, perto do teto do protocolo (int16 ×
  100 = ±327,67°) e longe de qualquer movimento real da PTU (±180°). Os
  gráficos dos relatórios partem de ±95° e só se expandem se os dados
  passarem disso (desktop e Android, mesma regra).
- **Giro contínuo longo (1.6.29).** Com a faixa ampliada, o teste de longa
  duração passou a fazer giros de centenas de graus e expôs um defeito antigo:
  o reaprendizado do bias (rede de segurança contra bias errado no boot) toma
  20 janelas seguidas a velocidade constante por "eixo parado sob bias
  errado". Um giro contínuo de mais de 20 s — 300° a 12°/s, ou 120° a 5°/s —
  fazia o firmware adotar a velocidade do giro como bias, apagar o que tinha
  integrado e girar ao contrário (simulado na 1.6.28: −480°, −170°, +400°).
  Agora, com o bias já confirmado, a troca só é aceita se couber na deriva
  física desde a última janela parada (`PAN_BIAS_RELEARN_MAX_JUMP_DPS` +
  `PAN_BIAS_SLOPE_MAX_DPS2` × tempo); os três giros erram menos de 0,1°
  (`sim/vaivem_sim.cpp`).
- **Teste de longa duração (`sim/soak_sim.cpp`), refeito.** O limite de ±90°
  zerava o erro acumulado a cada alvo além do curso e escondia três coisas: o
  defeito acima, as perdas de um reset do MPU no meio de um giro (sem a FIFO,
  o que gira até o firmware reconfigurar o chip não é amostrado) e o fato de
  que, sem referência absoluta, o erro dos giros se soma em 30 min. Agora:
  - a mesma bateria de sementes roda **sem e com falhas** (roteiro, ruído e
    falhas em fluxos aleatórios separados, então o caminho é o mesmo); as
    falhas não podem piorar a média do pior erro por semente em mais de 0,3°
    nem o percentil 90 em mais de 0,5° — medido em 160 sementes: +0,10 a
    +0,15° e +0,23 a +0,35°;
  - perdas inevitáveis (reset do MPU com o eixo em movimento, loop parado numa
    borda de giro) são medidas à parte, até 5° cada (maior medida: 1,1°);
  - tetos contra defeito que dispara: 8° acumulado e 3° + 1% do giro por
    trecho. O acúmulo normal, com o aquecimento forte do teste (até ±3°/s em
    5 min) e giros contínuos de até ~400°, chegou a 3,5° sem falhas e 4,7° com
    falhas em 160 sementes — é o limite físico do giro integrado descrito em
    "Os dois regimes de uso": em movimento contínuo o pan só é corrigido nas
    paradas;
  - 40 sementes por rodada (com 20 o percentil 90 oscila mais que o limite).
- **Escala.** Todos os giros desta sessão leram 1,0-2,0% longos, nos dois
  sentidos. Com as sessões calibradas em 0 (1.6.26 e 1.6.27), a escala
  necessária fica entre 1,022 e 1,033, média **1,027**. A estimativa de
  1,0487 da 1.6.25 veio de uma sessão que começou sem calibrar (pan em −1,0°).
- **Ainda em aberto:** nas duas sessões da 1.6.26, a volta horária −30 → 0
  perdeu ~1°; nesta sessão os giros horários vindos do lado negativo não
  mostraram isso. O CSV do `tools/diagnostico_pan.py` continua sendo o
  caminho para fechar a questão.

### Pan: a assimetria não se confirmou; o erro varia de giro para giro (v1.6.27)

Bancada com a 1.6.26 (PTU 0 → 12 → 20 → 30 → −12 → −20 → −30 → 0):
leituras 12,2 / 20,1 / 30,2 / −12,8 / −21,0 / −31,3 / volta ao zero −2,0
(e −1,5 pouco depois, parado).

| Giro | Lido ÷ real (1.6.26) | Convertido para a escala 1,030 |
|---|---|---|
| 0 → +30 (horário) | 1,007 | 0,997 |
| +30 → −12 (anti) | 1,024 | 1,001 |
| −12 → −20 (anti) | 1,025 | 1,003 |
| −20 → −30 (anti) | 1,030 | 1,008 |
| −30 → 0 (horário) | 0,977 | 0,967 |

Na sessão anterior (1.6.24, escala 1,030) foi horário 0,990 e anti-horário
0,980 e 0,977. Juntando tudo, horário e anti-horário não diferem além da
dispersão — e na MESMA sessão dois giros horários de 30° leram 0,997 e 0,967.
Um erro de escala, simétrico ou não, não muda de um giro para outro: o que
sobra (~±2% por giro, até ~1°) é outra coisa. A assimetria da 1.6.26 foi
desfeita (`PAN_SCALE_ASYM` = 0) e a escala média vai para 1,0425.

Indício da causa: a leitura mudou de −2,0 para −1,5 com o eixo PARADO. Isso
é o firmware corrigindo o bias para trás depois de parar (desfaz o que
integrou com o bias errado) — o zero do giro estava errado durante os
movimentos. Para separar de vez escala de bias, o próximo ensaio grava a
sequência com `tools/diagnostico_pan.py` (bias, taxa e pan interno a 2 Hz).

### Pan: erro ao voltar ao zero depois de vaivéns (v1.6.26)

Bancada com a 1.6.25: giros rápidos de ±30°, paradas curtas, vários
vaivéns — ao voltar ao zero, ~2,7° de erro (o tilt ficou bom e não mudou).

Causa: **assimetria de escala entre os sentidos** de giro. Nos giros de 30°
da sessão anterior, o horário leu 0,990 e os anti-horários 0,978 do giro
real: 1,2% de diferença. Um erro de escala simétrico se cancela na volta;
um assimétrico não — cada ida e volta de ±30° deixa ~0,7°, e 4 vaivéns dão
~2,9°. `sim/vaivem_sim.cpp` reproduz o número.

Correção: `PAN_SCALE_CORRECTION` = 1,0466 e `PAN_SCALE_ASYM` = −0,0059
(escala 1,0404 no horário e 1,0529 no anti-horário). Com o sensor assim, o
simulador volta ao zero a menos de 0,4° depois de 4 a 8 vaivéns. Os
simuladores passaram a gerar o giro com a mesma escala por sentido que o
firmware supõe. A assimetria vem de poucos giros (1 horário, 2
anti-horários): confirmar na bancada; se a volta ao zero errar para o outro
lado, a assimetria foi superestimada.

### Calibração de bancada na PTU (v1.6.25)

Placa montada na PTU-D300E, PRO-3600 em cima do cabeçote (sessões com a
1.6.23 e a 1.6.24).

**Tilt.** A PTU não serve de referência de 0,1°: em +20 ela marca 20,0 e o
PRO-3600 20,3. Contra o PRO-3600, o lado positivo ficou dentro de 0,3-0,4°
com 1,066; o negativo lia até 0,8° a menos com 1,042 (−29,3° para −30,1°).
`TILT_SCALE_CORRECTION_NEG` vai para **1,066** (mínimos quadrados: 1,0666),
igual ao lado positivo; o resíduo nos pontos medidos cai para ≤ 0,24°. A
assimetria antiga vinha de pontos só até −8,6°.

| PTU | PRO-3600 | 1.6.24 (NEG 1,042) | 1.6.25 (NEG 1,066, previsto) |
|---|---|---|---|
| −10 | −10,0 | −9,6 | −9,8 |
| −20 | −20,1 | −19,4 | −19,8 |
| −30 | −30,1 | −29,3 | −30,0 |

**Pan.** Giros de 30° com o tilt em 0: +30 → +29,7; 30 → 0 = −29,4;
0 → −30 = −29,3 — a leitura saía 1,8% curta nos três.
`PAN_SCALE_CORRECTION` vai de 1,030 para **1,0487**. Ainda são poucos giros
e um só tilt; o próximo passo é `tools/calibra_pan.py` com giros de ±45° a
±80° em vários tilts (preenche também `PAN_SCALE_COS2/_SIN2/_ASYM`).

**Zero do pan.** Na mesma sessão o pan começou em −1,0° com a PTU em 0
(e −0,7° na volta): o pan não tinha sido recalibrado depois dos movimentos
do tilt. Parte do movimento do tilt "vaza" para o pan pela sensibilidade
cruzada do giro (±2% no MPU6050). Antes de um ensaio de pan, calibrar com
a PTU em 0.

### Os dois regimes de uso: parado por muito tempo e em movimento constante (v1.6.24)

O pan e o tilt podem ficar parados por horas ou se mover sem parar. Os dois
regimes passaram a ser simulados juntos, nos dois eixos
(`sim/regime_sim.cpp`: giro aquecendo 0,3 °/s em ~10 min — partida a frio,
pior caso —, zero passeando, ruído e 3% de erro de escala):

| Regime (30 min, pan ±60° a 10 °/s, tilt ±20° a 5 °/s) | Pan: pior erro | Tilt: pior erro |
|---|---|---|
| Parado 2 h | 0,03° | 0,02° |
| Varrendo, 5 s parado em cada ponta | 2,0° | 1,8° (*) |
| Varrendo, 3 s parado em cada ponta | 7,2° | 1,9° (*) |
| Varrendo **sem parar nunca** | **37°** | 2,2° (*) |
| Só o tilt varre sem parar (pan fixo) | **satura (±90°)** | 2,2° (*) |
| Só o pan varre sem parar (tilt fixo) | **37°** | 0,03° |

(*) em movimento; é o atraso do filtro da tela (~0,3 s a 5 °/s), igual com
ou sem a fusão. Parado, o tilt volta ao valor exato.

**Tilt — não depende de parar.** Ele tem a gravidade como referência o tempo
todo. A simulação achou dois defeitos da fusão da 1.6.23, corrigidos aqui:

1. A fusão usava o bias do giro do `PanSensor` continuamente. Só que o
   `PanSensor` cuida do bias no eixo do **pan**; a componente no eixo do tilt
   ele deixa andar, e com o tilt em movimento constante ela chegou a 1,3 °/s
   de erro — 1,3° a mais de erro no tilt (4,5° contra 1,8° do acelerômetro
   sozinho). Agora o bias é copiado **uma vez**, ao armar a fusão, e quem
   acompanha a deriva é o termo integral da própria fusão
   (`TILT_FUSION_KI` = 0,25: um erro de bias some em ~5 s), parado ou em
   movimento.
2. O bias aprendido era zerado a cada leitura perdida no I2C; com falhas
   frequentes ele nunca convergia. Agora só é descartado quando giro e
   acelerômetro divergem.

**Pan — precisa de paradas; é o limite físico do método.** O pan é a
integral do giroscópio e não tem referência absoluta (não há bússola). Parado,
o ZUPT cancela a deriva: 2 h parado, 0,03°. Em movimento, o zero do giro só é
corrigido quando o eixo para por alguns segundos (janela de 1 s inteira
parada); entre uma parada e outra, o que o zero do giro andar vira erro de
ângulo. Sem parada nenhuma, mesmo com o chip já quente, sobra o resíduo de
~0,005 °/s (≈ 0,3 °/min). Regras práticas:

- ligar e esperar ~10 min antes de medir (o grosso do aquecimento);
- em movimento contínuo, parar **5 s ou mais** de tempos em tempos — pan E
  tilt parados juntos (se só o tilt se move, o pan também não é corrigido);
- para pan absoluto em movimento contínuo, o caminho é outra referência: o
  encoder da própria PTU (RS-485) ou um magnetômetro. Compensar o bias pela
  temperatura do chip reduziria a deriva, mas precisa de dados desta unidade
  (captura parada de 30 min a partir do frio).

## Azimute (pan) pelo giroscópio (v1.2.0)

O acelerômetro mede a direção do vetor gravidade — e girar em torno da
vertical **não muda esse vetor**. Ele é fisicamente cego ao pan; não é
questão de código. Mas o MPU6050 é 6-DOF: o **giroscópio** que já estava na
placa (e que o firmware não configurava nem lia até a v1.1.0) mede
velocidade angular em torno de qualquer eixo, inclusive o vertical.
Integrando essa taxa sai o ângulo de pan relativo ao zero calibrado — mesmo
paradigma que o tilt já usa. **Nenhum hardware novo foi adicionado.**

O problema clássico de integrar giro é o drift: um bias residual vira uma
rampa de graus por minuto. O que torna a abordagem confiável aqui é o padrão
de uso — o eixo de pan fica **parado a maior parte do tempo** e se move em
rajadas de poucos segundos — combinado com **ZUPT**. Quatro mecanismos
trabalham juntos em `PanSensor.h/.cpp` (cada um documentado em detalhe no
cabeçalho do header):

1. **Compensação de tilt.** O sensor está na parte que inclina, então o eixo
   Z dele não aponta pra vertical. Pegar `gz` cru subestimaria a taxa por
   `cos(tilt)` — a 60° de tilt a leitura sairia pela metade. A taxa correta é
   a projeção sobre a vertical escrita em coordenadas do corpo:
   `ω_pan = g·u` (giro dos **três** eixos vezes a vertical unitária `u = a/|a|`
   do acelerômetro, desde a 1.6.17; antes, `gz·cos(θ) + gy·sin(θ)`, que
   supunha o eixo X na horizontal), com `u` vindo do **mesmo burst I2C**
   (até a 1.6.14 o sinal do termo de `gy` estava trocado — ver "Ensaio de
   bancada −30/+90/+30/+60/0" abaixo). Essa projeção dá positivo no sentido
   **anti-horário** visto de cima; o pan do usuário é positivo no sentido
   **horário** visto de cima, por isso o firmware a multiplica por
   `PAN_DIRECTION_SIGN = −1` (v1.6.16).
   Por ser uma projeção (produto escalar), e não a fórmula de taxa de Euler
   `(gy·sinφ + gz·cosφ)/cosθ`, não há singularidade em nenhum tilt.
2. **ZUPT.** A cada 1s, se a **média** de `gy`/`gz` na janela estiver perto
   do bias corrente, a janela é dada como parada e o bias é refinado. Usar a
   média (e não o pico) deixa o detector imune a vibração, que é de média
   zero: o mastro pode estar balançando sob vento que a janela ainda é
   corretamente reconhecida como parada. O bias é estimado **por eixo, no
   referencial do sensor**, e só depois projetado — o bias de fábrica é fixo
   nos eixos do chip, e projetado ele mudaria com o tilt (ver a limitação
   corrigida na 1.6.1, mais abaixo). Janela fora do limiar é movimento e
   **não ensina nada ao bias** (ver "Giro lento, feito à mão" abaixo).
3. **Cancelamento de janela parada.** O que foi integrado dentro de uma
   janela classificada como parada é subtraído de volta — parado, o ângulo
   fica cravado, sem random walk.
4. **Fator de escala** (`PAN_SCALE_CORRECTION`). Resolvido o bias, o erro
   dominante vira a tolerância de sensibilidade do giro (~±3% de fábrica).
   Ele é proporcional ao **deslocamento atual** em relação ao zero — não se
   acumula com o tempo nem com o número de movimentos — e some quando o eixo
   volta ao zero.

   Os 24 pontos de bancada obtidos com a mesa Mitutoyo AVB007451 (v1.6.7)
   eram na verdade um teste de **inclinação** com o inclinômetro Mitutoyo
   PRO3600, não de azimute — ver `TILT_SCALE_CORRECTION` na seção
   "Estabilidade da leitura" abaixo, e `PAN_SCALE_CORRECTION` voltou a `1.0`
   por uma versão (v1.6.8).

   **Calibração PRELIMINAR na 1.6.9**, com só 2 pontos contra mesa
   giratória/goniômetro com escala angular: alvo 8,00° → lido 7,75°; alvo
   9,00° → lido 8,75°. Erro crescendo com o deslocamento (assinatura de
   escala, não de offset), e batendo com a tolerância de fábrica do giro.
   Ajuste por escala pura: leitura = 0,9707 × referência.
   `PAN_SCALE_CORRECTION` foi de `1,0` para `1,030` — reduz o resíduo nos
   dois pontos de ±0,25° para ±0,02°. Diferente do tilt (24 pontos contra um
   inclinômetro certificado ±0,02°), aqui é só 2 pontos contra uma
   referência de precisão não quantificada: **refinar com mais pontos**
   (e idealmente mais afastados de zero, ex.: 45°/90°) quando possível.

### Validação feita até agora

O `PanSensor.cpp` real foi compilado contra um MPU6050 falso e exercitado com
sinais sintéticos (bias de fábrica de 5°/s, ruído, vibração, tilt fixo):

| Cenário | Resultado |
|---|---|
| Parado 60s com bias de 5°/s | 0,003° de deriva |
| Movimento de 60° e 30s parado depois | 60,003° |
| Parado 60s sob vibração de 3°/s de pico | 0,004° de deriva |
| 60° de pan com o sensor a 60° de tilt | 60,02° (sem a compensação daria 30°) |
| 40 movimentos de ±45°, voltando ao zero | 0,066° de erro residual |
| **1.6.1:** placa parada, tilt muda 0→45° (bias típico) | 0,00° (na 1.6.0: travava em −90° em ~1 min) |
| **1.6.1:** pan +30°, tilt 0→45° e volta a 0° | 29,76° mantido (na 1.6.0: −39,5°, sem recuperar) |
| **1.6.1:** bias de ±15°/s, tilt 0→60°, pan +30°, tilt 60→10° | 29,75° mantido |
| **1.6.2:** liga com a placa em movimento, tilt 0→45°, pan +30° | 29,75° (na 1.6.1: travava em −90°) |
| **1.6.2:** giro constante de 5°/s durante o boot, tilt 0→45° | deriva desfeita em ~22 s, 0,00° (na 1.6.1: −90° travado) |
| **1.6.2:** varreduras do motor −80° → +80° a 20°/s | 80,04°, sem reaprendizado indevido |

O piso de detecção medido bate com o previsto (`limiar × janela` = 1°/s × 1s):
movimentos de até ~1° são descartados como ruído, e a partir de ~2° são
preservados integralmente. *(Valores da 1.2.0; o limiar caiu para 0,1°/s
na 1.6.14 — ver a seção seguinte.)*

### Giro lento, feito à mão (v1.6.14)

Em bancada, girando o eixo à mão, o pan **leu negativo (−0,6°) depois de um
giro positivo** e, quando lia certo, **demorava muito para estabilizar**.
A causa era o ZUPT: com limiar de 1°/s, um giro à mão (lento, com partida e
parada suaves) caía janela a janela abaixo do limiar — cada janela era tida
como parada, o que fora integrado nela era **apagado**, e 10% da velocidade
do giro era **ensinada ao bias** a cada janela. Depois de parar, o bias
errado puxava a leitura para o lado contrário do giro, e ela levava perto de
um minuto para voltar enquanto o bias se corrigia. Reproduzido com o
`PanSensor.cpp` real (`sim/pan_sim.cpp`, cenários K-O):

| Cenário | 1.6.13 | 1.6.14 |
|---|---|---|
| K: giro à mão +8° em 4 s | 6,39°, oscilando ~0,19° por ~40 s | 7,98°, estável |
| L: +8° em 16 s (pico 1°/s) | **0,00°**, mínimo −0,64° | 7,85°, nunca negativo, estável |
| M: 8 passos de +1° (1,5 s cada) | **0,00°**, mínimo −0,53° | 7,95°, estável |
| N: +8° em 32 s, depois −8° em 4 s | −6,12° | −0,34° |
| O: +8° em 4 s sob vibração de 2°/s a 33,3 Hz | **+27,74°** (dispara) | 0,00° (não dispara) |

O que mudou em `PanSensor::closeWindow()`:

1. **Limiar de "parado" de 1°/s para 0,1°/s** (`PAN_ZUPT_RATE_THRESHOLD_DPS`),
   ainda acima do ruído da média de uma janela e do atraso com que o filtro
   de bias acompanha a pior deriva térmica simulada.
2. **Janela de movimento não ensina o bias** — era ela que o contaminava.
3. **Bordas do giro.** Partida e frenagem suaves ainda passam algumas
   janelas abaixo de 0,1°/s. Elas são reconhecidas pela forma (mesmo sentido
   do giro, velocidade crescendo ou caindo janela a janela — a deriva térmica
   não forma essa rampa): na partida, quando o giro é detectado, o que foi
   cancelado nelas **volta** e o que elas ensinaram ao bias é **desfeito**;
   na frenagem, elas continuam sendo giro (`PAN_ZUPT_EDGE_*`).
4. **Limiar adaptativo à vibração** (`PAN_ZUPT_VIBRATION_FACTOR`). Sob
   vibração a média de uma janela não zera — e a mediana anti-picos
   ("LEITURAS ESPÚRIAS" em `PanSensor.h`) piora isso: perto de submúltiplos
   da amostragem de 100 Hz (33 Hz = 100/3, 14,3 Hz = 100/7) ela **retifica**
   a vibração num bias falso que muda devagar. Um limiar fixo de 0,1°/s
   falhava no soak de 100 sementes (até 2,5°). O limiar efetivo cresce com a
   vibração medida na janela — pela diferença entre amostras brutas
   sucessivas, que o próprio giro (mesmo acelerando à mão) quase não move, e
   descontado o ruído do sensor. Sem vibração fica nos 0,1°/s; sob vibração
   forte se aproxima do 1°/s antigo.
5. **Deriva do bias limitada ao fisicamente possível**
   (`PAN_BIAS_SLOPE_MAX_DPS2`, 0,05 → 0,015°/s²): uma deriva falsa aprendida
   sob vibração forte era extrapolada por 30 s depois de parar e chegou a
   levar a leitura a dezenas de graus (cenário O).
6. **Adoção do bias** no boot/calibração usa a média das duas janelas
   coerentes (`PAN_BIAS_ADOPT_COHERENCE_DPS`, ainda 1°/s, para não travar sob
   vibração).
7. **Rede de segurança** (reaprender o bias): antes bastavam 20 janelas
   vizinhas "coerentes" a 1°/s — e um giro à mão lento e suave passava nesse
   teste e era apagado. Agora as 20 médias têm de cair sobre uma **reta**
   (bias constante ou derivando), com resíduo de 0,05°/s quando o bias já foi
   confirmado parado (`PAN_BIAS_RELEARN_RESIDUAL_DPS`), maior sob vibração.

Os cenários A-J (motor, mudanças de tilt, boot em movimento, leituras
espúrias) continuam passando, e no soak de **100 sementes** (motor, deriva
térmica, vibração de 5-15 Hz e todas as falhas de hardware) o resultado é o
mesmo da 1.6.13: mediana 0,30°, p90 0,63°, pior 0,96°, nenhuma semente mais
de 0,1° pior.

**Limites que sobram:**

- um giro extremamente lento — da ordem de 8° em 30 s ou mais, sem nenhum
  trecho acima de ~0,1°/s por alguns segundos — se confunde com deriva
  térmica nas bordas e pode perder alguns décimos de grau (cenário N). Em
  bancada, girar em poucos segundos;
- **sob vibração forte** (ex.: ≥1°/s acima de ~10 Hz no eixo), o limiar sobe
  e um giro à mão lento volta a se perder como na 1.6.13 — agora sem
  disparar (cenário O). O motor (20-30°/s) não é afetado;
- abaixo de ~4 Hz, balanço do eixo é deslocamento angular de verdade e é
  integrado (com média zero).

**Isso valida a matemática, não o hardware.** Falta rodar contra o MPU6050
real — ver as duas confirmações de bancada nas limitações abaixo.

### Ensaio de bancada −30/+90/+30/+60/0 (v1.6.15)

Com a 1.6.14, em bancada, girando à mão para posições conhecidas:

| Real | −30° | +90° | +30° | +60° | volta a 0° |
|---|---|---|---|---|---|
| Lido (1.6.14) | −35,6° | +83,5° | +20,2° | +49,1° | −7,5° |

Erro que **cresce com o tempo**, não com o ângulo: o zero do giroscópio
anda com o eixo parado (aquece por minutos depois de ligar, passeia), e o
limiar de "parado" de 0,1°/s da 1.6.14 não acompanhava. Bastava o zero andar
um pouco mais que 0,1°/s entre duas janelas paradas para **todas** as
seguintes saírem do limiar; como janela em movimento não ensina o bias, ele
nunca mais voltava, e a diferença era integrada sem fim. A 1.6.13 (limiar de
1°/s) não tinha esse problema, mas perdia os giros lentos. Três correções e
duas proteções em `PanSensor.cpp`/`Mpu6050.cpp`:

1. **Bias que andou com o eixo parado** (`PAN_ZUPT_SHIFT_*`): 6 janelas
   seguidas sem giro de verdade (abaixo de 2°/s) com médias sobre uma reta
   lisa e pouco inclinada — ou 20 seguidas abaixo de 1°/s, como a 1.6.13
   tratava — só podem ser o eixo parado sob um bias que mudou. O bias é
   readotado e o que essas janelas integraram é desfeito.
2. **Limiar adaptado ao próprio giro** (`PAN_ZUPT_WANDER_*`): o passo da
   média entre janelas paradas é medido o tempo todo, e o limiar não fica
   abaixo de 4× esse passo. Sensor estável: 0,1°/s; sensor que passeia: sobe
   na medida.
3. **Bias acompanha mais rápido** (`PAN_ZUPT_BIAS_ALPHA` 0,1 → 0,3).
4. **Sinal da compensação de tilt corrigido**: `ω_pan = gz·cos θ + gy·sin θ`.
   O sinal trocado multiplicava a taxa por `cos(2θ)` — 6% a menos a 10° de
   tilt, metade a 30°, zero a 45° e o sentido invertido acima disso. Os
   simuladores tinham o mesmo sinal trocado no sensor falso e por isso não
   pegavam; agora usam a física (`gy = Ω·sin θ`).
5. **Fundo de escala do giro ±250 → ±1000°/s**: girar 90° à mão em meio
   segundo passa de 350°/s no pico. As amostras saturadas eram descartadas e
   o trecho integrado com a taxa de antes/depois (simulado: +90° rápido lia
   79-82°, com o erro somando giro a giro). Com o fundo de escala maior, um
   quadro de lixo do I2C traz giro de até 1000°/s; amostra com salto
   impossível em 10 ms (`PAN_GYRO_SLEW_MAX_DPS`, 100°/s) é descartada. E o
   chip agora é configurado **antes** de acordar: depois de um reset ele
   volta em ±250°/s, e acordado com uma escrita do meio falhando mediria o
   giro 4× maior até a checagem seguinte.

`sim/pan_sim.cpp`, com o ensaio acima e o zero do giro andando:

| Cenário | 1.6.14 | 1.6.15 |
|---|---|---|
| P: −30/+90/+30/+60/0, zero passeando (0,02°/s/√s) | −30,0/88,7/27,0/56,2/−2,5 (pior 3,8°) | pior 0,17° |
| Q: idem, logo depois de ligar (zero +2°/s em ~1 min) | −17,7/90/49,2/90/54,5 (pior 54°) | pior 0,10° |
| R: tilt 30° fixo, pan +40° | 20,0° | 40,0° |

Avaliação com 20 sementes do mesmo ensaio (pior erro das 5 leituras,
mediana / p90), sensor com DLPF, quantização e saturação reais:

| Condição | 1.6.13 | 1.6.14 | 1.6.15 |
|---|---|---|---|
| giro estável, tilt 0° | 0,09 / 0,52° | 0,02 / 0,07° | 0,02 / 0,07° |
| giro estável, tilt 30° | 45 / 45° | 45 / 45° | 0,02 / 0,07° |
| zero passeando 0,02°/s/√s | 0,25 / 0,61° | 3,6 / 14° | 0,22 / 0,45° |
| zero passeando 0,03°/s/√s | 0,23 / 0,42° | 11 / 20° | 0,44 / 0,88° |
| aquecimento +2°/s em ~1 min | 0,32 / 0,50° | 48 / 75° | 0,16 / 0,29° |
| giro à mão +8° em 4 s | perde 1,6° | 0,01° | 0,01° |
| giro à mão +8° em 16 s | perde tudo | 0,04° | 0,04° |
| 8 passos de +1° | perde tudo | 0,03° | 0,03° |

Os cenários A-O continuam passando (N agora é +8° em 16 s: mais lento que
isso é indistinguível de um zero que andou). No soak de 100 sementes (motor,
deriva térmica, vibração, todas as falhas de hardware, agora com o sensor
emulado no sinal físico e com o fundo de escala do registrador): mediana
0,30°, p90 0,65°, pior 1,05° — 99 de 100 sementes dentro de 1°.

**Limites que sobram:**

- giro à mão contínuo e muito lento — velocidade quase constante abaixo de
  ~0,5°/s por vários segundos, ou abaixo de 1°/s por mais de ~20 s — é
  tomado como zero que andou e apagado (+8° em 24 s já se perde);
- com o zero do giro passeando forte, o limiar sobe e giros lentos voltam a
  perder um pouco, e entre duas leituras sobram alguns décimos de grau
  (tabela acima). Em bancada: ligar e esperar 1-2 min antes de calibrar, e
  girar em poucos segundos;
- `PAN_SCALE_CORRECTION` (1,030) foi calibrado na 1.6.9 com o ZUPT antigo e
  o sinal trocado; refazer a calibração com giros de 90° no tilt 0°.

### Sentido do pan e aproximação lenta (v1.6.16)

Ensaio de bancada com a 1.6.15: **o sentido do pan saiu invertido**
(movimento positivo lido negativo) e **sobrou um erro de ~6° somado ao longo
dos giros**, que depois ficava constante.

1. **Sentido.** A projeção corrigida na 1.6.15 dá positivo no sentido
   anti-horário visto de cima (regra da mão direita em torno da vertical),
   qualquer que seja a montagem; o pan do usuário é positivo no horário,
   como azimute de bússola. Novo `PAN_DIRECTION_SIGN = −1`, aplicado em
   `projectedDps()`, por onde passa toda conta que soma ao ângulo. A 1.6.14
   acertava o sentido por acaso: com o sensor montado mais "em pé" que
   deitado (`atan2(ay, az)` entre 45° e 135°), o sinal trocado de `gy`
   invertia o sentido de novo. Cenários S1/S2 de `sim/pan_sim.cpp`: giro
   horário de 30° com a placa deitada e em pé — 1.6.15: −30,0°; 1.6.16:
   +30,0°.
2. **Erro de ~6°.** Ao readotar um bias que andou (`PAN_ZUPT_SHIFT_*`, 1.6.15),
   as janelas abaixo de 2°/s da excursão eram apagadas inteiras como deriva.
   Mas a **aproximação final da marca**, feita devagar à mão logo depois do
   giro, também fica abaixo de 2°/s: cada readoção depois de um giro levava
   junto a aproximação. Agora janela lenta a até 3 janelas de um giro, no
   mesmo sentido dele e acima do nível do bias errado, é giro — dela sai só
   a parte do bias. A parte do bias em cada janela vem da reta ajustada no
   fim e, antes dela, de uma interpolação até o começo da excursão (quando
   o bias ainda estava certo), em vez de uma extrapolação da reta. Nunca sai
   mais do que a janela integrou (no fim do curso o integrador fica preso).
3. A readoção forçada (20 janelas abaixo de 1°/s) agora exige médias ainda
   perto de uma reta: passos de 1° à mão a cada 2,5 s ficavam 20 s abaixo de
   1°/s e eram apagados. A inclinação máxima da reta subiu de 0,04 para
   0,08°/s² (um zero que passeia forte chega a 0,05°/s²).

| Cenário (`sim/pan_sim.cpp`) | 1.6.15 | 1.6.16 |
|---|---|---|
| S1: giro horário de 30°, placa deitada | −30,0° | +30,0° |
| S2: idem, placa em pé (tilt 90°) | −30,0° | +30,0° |
| T: −30/+90/+30/+60/0 com os últimos 3° de cada giro em 5 s, zero passeando | pior 4,5° (volta ao zero: −4,1°) | pior 1,2° |

Avaliação com 20 sementes (pior erro das 5 leituras, mediana / p90), com
a aproximação final lenta e o zero passeando 0,02°/s/√s:

| Aproximação | 1.6.15 | 1.6.16 |
|---|---|---|
| últimos 2° em 3 s | 2,0 / 3,0° | 0,4 / 0,9° |
| últimos 3° em 5 s | 3,1 / 4,7° | 0,6 / 2,1° |
| últimos 5° em 8 s | 5,3 / 9,0° | 0,9 / 5,3° |

Sem a aproximação lenta, o resultado é o da 1.6.15 (comparando no mesmo
sentido de giro: soak de 100 sementes idêntico, mediana 0,30°, p90 0,65°).
Com o sentido novo, o soak dá mediana 0,29°, p90 0,71°, pior 1,30° — 99 de
100 sementes dentro de 1°; a diferença é só o acaso de como a deriva cai em
relação ao sentido dos giros.

**Limite que sobra:** aproximação muito lenta com o zero do giro andando
ainda perde parte dela (casos de 5° em 8 s, acima). Em bancada: chegar na
marca com um giro só, sem ficar corrigindo devagar; se precisar corrigir,
parar 2-3 s antes.

### Pan nos três eixos e sentido do tilt (v1.6.17)

Ensaio de bancada com a 1.6.16: **o tilt lia negativo ao subir**, e **o pan
na posição 40 mostrava 28,75°**.

1. **Pan: 0,72 do real = montagem.** A projeção usava só `gy` e `gz`
   (`gz·cos θ + gy·sin θ`, `θ = atan2(ay, az)`), supondo o eixo X do chip na
   horizontal, ao longo do eixo do tilt. Com o X a φ da horizontal, a parte
   do giro de pan que cai no eixo X ficava de fora: a taxa saía × cos φ, e
   28,75/40 = cos 44°. A montagem real já tinha se mostrado diferente da
   prevista (o sentido do pan na 1.6.16, a correção de escala do tilt de
   4-7%). Agora a projeção é nos três eixos, `ω_pan = g·u` com
   `u = a/|a|`, e o bias é estimado nos três — vale para qualquer montagem.
   O cenário U de `sim/pan_sim.cpp` monta o sensor com o X a 44°: a 1.6.16 lê
   **28,75°**, a 1.6.17 lê **40,0°**.
2. **Movimento de tilt no ZUPT.** Com o eixo X na conta, o giro do próprio
   tilt passa a aparecer nas janelas. Ele é reconhecido pelo acelerômetro (a
   vertical gira junto, `(u1 × u0)/T`, explicando ao menos metade do giro
   fora da vertical) e fica fora da reta do reaprendizado de bias (cenário G:
   bias de boot errado e mudança de tilt no meio).
3. **Tilt: `TILT_DIRECTION_SIGN = −1`**, aplicado só no ângulo reportado.
   Os lados NEG/POS da correção de escala continuam escolhidos pelo sinal do
   ângulo **bruto**, que é o que a assimetria do sensor segue.
4. **Diagnóstico:** a characteristic de diagnóstico ganhou o eixo X e a
   vertical (75 bytes; apps antigos leem os 55 primeiros), e
   `python-app/tools/diagnostico_pan.py` mostra a coluna **"X fora"** — o
   ângulo do eixo X do chip fora da horizontal. Com a placa parada, ela diz
   na hora se a montagem é a que explica os 28,75°.

Resultados: `sim/run.sh` completo OK. Soak de 100 sementes: mediana 0,34°,
p90 0,71°, pior 1,58° — com a montagem prevista (X horizontal) o eixo X só
acrescenta ruído e deriva à conta (comparando no mesmo sentido de tilt:
mediana 0,29 → 0,32°, pior 1,30 → 1,37°); é o custo de valer para qualquer
montagem.

## Modo Vibração no eixo de pan (v1.3.0)

A captura de vibração agora amostra os dois eixos. Mas eles carregam
**grandezas diferentes**, e a razão é o próprio ZUPT:

- **tilt**: o ângulo relativo à calibração, em graus;
- **pan**: a **velocidade angular**, em graus/s.

Guardar o ângulo de pan integrado não funcionaria. O ângulo de pan sai da
integração com ZUPT, e o mecanismo 3 acima *cancela de propósito* o que foi
integrado enquanto o eixo está parado — que é exatamente a condição de um
ensaio de vibração. O firmware apagaria o sinal que se quer medir. A
velocidade angular não passa por integração nem por cancelamento, e ainda
deixa o bias residual concentrado em 0 Hz, onde a análise espectral já o
descarta por construção.

Os apps integram (regra do trapézio) e removem a tendência linear — o bias é
uma constante somada à taxa, e integrar constante dá exatamente uma rampa,
que a reta ajustada elimina.

### Sentido do tilt de volta ao original (v1.6.18)

Com a 1.6.17 gravada, **o tilt positivo aparecia negativo** no app — a
inversão feita na 1.6.17 (item 3 acima) deixou o sentido errado para a
montagem atual. `TILT_DIRECTION_SIGN` voltou a **+1**. Só o ângulo reportado
muda (tela, extremos, Modo Vibração, Modbus e BLE); os lados NEG/POS da
correção de escala seguem o sinal do ângulo bruto, como antes. Conferir em
bancada depois de gravar: subir tem de ler positivo.

### Pan: desnível da viatura e balanço do mastro (v1.6.19)

Revisão do modelo de cálculo do pan em busca dos erros que sobravam. O
simulador ganhou um modo de **corpo rígido** (`sim/pan_sim.cpp`, cenários
V): cinemática completa do pan-tilt, com o eixo de pan podendo estar fora da
vertical, a placa montada torta, balanço do mastro que gira o corpo de
verdade e o ruído real do acelerômetro (~2,3 mg RMS). Ele expôs dois erros
que os cenários anteriores (mastro sempre vertical) não tinham como ver:

1. **Desnível (eixo de pan fora da vertical).** A projeção ω_pan = g·u é
   exata só com o eixo de pan na vertical. Inclinado de β, o eixo do tilt
   deixa de ser horizontal e g·u = ω_pan·cos β' + ṫ·(â·u): até sen β de todo
   movimento de tilt vaza para o pan. Correção: o eixo do tilt â é fixo nos
   eixos do chip e perpendicular ao giro de pan, então a taxa é projetada em
   n = (u − (u·â)â)/|…|, o que zera o vazamento (resta 1 − cos β: 0,14% a
   3°). â é aprendido das janelas de tilt (autovetor principal de
   Σ dev·devᵀ·T) e o que vazou antes de ele valer é descontado de uma vez.
   Com o mastro vertical, n = u: nada muda.
2. **Balanço do mastro.** Rotação em torno de eixos horizontais não tem
   componente no pan, mas o resíduo dela na média de 1 s fazia o ZUPT
   achar que nenhuma janela estava parada — o bias derivava, e abaixo de
   ~1 Hz nem chegava a ser adotado no boot (leitura parada em 0°). O
   acelerômetro vê o balanço (a vertical gira junto, ω⊥ = u × u̇): a janela
   passa a ser parada para o pan quando a componente ao longo de n está
   abaixo do limiar e o resíduo horizontal não explicado pelo acelerômetro é
   pequeno; o bias aprende a parte vertical direto e a horizontal como
   (giro − rotação vista), e a adoção no boot também desconta a rotação
   vista. A rotação vista usa blocos de 8 amostras nas pontas da janela
   (ruído ~2,5x menor). Sem rotação vista (bias andando), a janela segue o
   caminho de readoção de sempre.

| Cenário (sim/pan_sim.cpp) | 1.6.18 pior / final | 1.6.19 pior / final |
|---|---|---|
| V1 — desnível 3°, tilt 0→60, pan +30, tilt volta | 3,17° / 0,38° | 0,77° / 0,02° |
| V2 — desnível 5°, pan girando com o tilt inclinado | 3,04° / −2,54° | 0,66° / −0,03° |
| V3 — V1 com balanço 1,5°/s a 1,2 Hz | 8,17° / 2,63° | 0,79° / 0,24° |
| V3b — nivelado, só balanço | 4,87° / 2,04° | 0,29° / 0,13° |
| V5 — 10 min, desnível 3°, balanço 2°/s a 0,6 Hz | 60° (não integrava) | 0,92° |
| V5 — idem a 1,2 Hz / 2,5 Hz | 18,8° / 3,4° | 0,59° / 0,62° |
| V6 — desnível 10° (transitório da 1ª janela de tilt) | 10,45° / 0,94° | 2,54° / −0,03° |
| V4 — parado 10 min (ruído do acelerômetro) | 0,02° | 0,02° |

Os 0,77° do V1 são o transitório da primeira janela de tilt, antes de o eixo
ser aprendido (descontado no fim dela). Nos ensaios de bancada sem desnível
nem balanço (A–U, e P e T comparados semente a semente em 60 realizações do
ruído), o resultado é idêntico ao da 1.6.18. P e T passaram a ser checados
em 10 sementes (média e máximo), e não numa realização só.

**Pendente em bancada:** confirmar com a base inclinada de propósito (3–5°,
calço sob um lado) que um tilt de 60° não move mais o pan, e que o pan fica
estável com a estrutura balançando.

### Um detalhe que quase virou bug

A detecção de frequência dominante compara o pico com a **mediana** do
espectro, o que só é justo se o piso de ruído for plano. No tilt é: o ruído
do acelerômetro é branco no ângulo. No pan **não é**: o ruído do giroscópio é
branco na *taxa*, e integrar ruído branco dá um passeio aleatório, cujo
espectro cai com 1/f². Nessa forma a mediana global fica dominada pelas
frequências altas, e qualquer bin de baixa frequência vira um "pico" enorme.

Em teste, com a primeira versão do código, **ruído puro era apontado como
frequência dominante em 100% das tentativas** (0,37 Hz, com folga de SNR).
A correção foi rodar a detecção do pico no espectro da *taxa* — onde o ruído
é branco de verdade — e converter só a amplitude do pico para graus, por
`A = R(f) / (2·pi·f)`. Por isso as amostras de pan guardam as duas séries: a
taxa (para a detecção) e o ângulo integrado (para os gráficos no tempo e as
estatísticas). Depois da correção: 0 falsos positivos em 48 capturas de
ruído puro em 4 durações diferentes, mantendo 30/30 de detecção de sinal
real, inclusive com amplitude de 0,02°.

## Sensibilidade da leitura contínua (v1.4.0)

O filtro da leitura contínua deixou de ser uma média móvel exponencial de
constante fixa (0,5s) e passou a ser um **filtro adaptativo "1-euro"**
(Casiez et al.): a frequência de corte sobe quando o ângulo está mudando de
verdade e desce quando o eixo está parado.

O motivo é que, com constante de tempo fixa, estabilidade e rapidez são um
cabo de guerra — mais suave é sempre mais lento. Separando os dois regimes dá
para ganhar nos dois. Medido em simulação da cadeia completa (ruído do
MPU6050 → DLPF de 21Hz → `atan2` → filtro), com o eixo parado e depois com
uma rampa real de 10°:

| | média móvel (0,5s) | 1-euro adaptativo |
|---|---|---|
| Ruído com o eixo parado | 0,016° RMS | **0,005° RMS** |
| Atraso p/ acompanhar 90% de um movimento | 0,63 s | **0,21 s** |

Ou seja, ~3x menos ruído **e** ~3x mais rápido — não é troca.

**O degrau de exibição continua em 0,25°**, o mesmo da versão 1 — é requisito,
e mantém a comparação entre as duas versões direta. (Desde os apps 2.2.0 há
também a opção de 0,1° por eixo, para bancada — ver `python-app/README.md`;
o ruído de 0,005° RMS continua muito abaixo de meio degrau.) Com esse degrau a tela já
era estável com o filtro antigo (meio degrau é muito maior que o ruído), então
o ganho do filtro novo **não aparece na estabilidade da tela**: aparece em
duas outras coisas.

**1. A tela assenta mais rápido no degrau certo** depois de um movimento:
1,75s → 1,50s no ruído nominal, e 1,75s → 0,75s com 3x o ruído.

**2. O mín/máx ficava mais limpo** — 0,120° → 0,040° de faixa falsa numa
sessão de 10 min com o eixo parado. Esse segundo ponto **deixou de valer na
v1.5.0**: o mín/máx não sai mais deste filtro, e sim do caminho de medida
(ver "Extremos medidos no firmware" abaixo). O motivo é que a leitura da tela,
por mais limpa que fique, é uma leitura *suavizada* — ela subestimava rajadas
reais, que é o erro que importa num laudo de carga de vento.

### De onde vem o ruído, e o que NÃO adiantaria mexer

| Etapa | Contribuição |
|---|---|
| Acelerômetro após o DLPF de 21Hz | 2,2 mg |
| Ângulo cru, via `atan2` | 0,127° RMS |
| Após o filtro adaptativo | **0,005° RMS** |
| Quantização do protocolo (0,01°) | 0,003° — desprezível |

Duas coisas que pareceriam melhorias e não são:

- **Baixar o DLPF do MPU6050** (de 21Hz para 5Hz) cortaria o ruído pela
  metade antes mesmo do filtro. Mas o DLPF é compartilhado com o Modo
  Vibração, que precisa da banda até ~5Hz e usa os 21Hz como anti-aliasing
  a 50 amostras/s. Cortar ali destruiria o ensaio de vibração.
- **Aumentar a resolução do protocolo** (`ANGLE_SCALE` acima de 100) não
  ajuda: a quantização de 0,01° já contribui menos que o ruído do sensor.

O caminho que sobra, se um dia precisar de mais, é reduzir o
`ANGLE_FILTER_MIN_CUTOFF_HZ` — ao custo de o filtro demorar mais para
assentar depois de um movimento.

## Extremos medidos no firmware (v1.5.0)

Até a v1.4.0 o mín/máx era calculado nos apps, a partir das leituras que eles
recebiam. Isso tinha dois problemas somados, e o resultado ia direto para o
relatório:

1. **O valor lido era o da tela.** O filtro da leitura contínua é pesado de
   propósito — o display tem degrau de 0,25° e não pode tremular. Mas um filtro
   pesado achata pico: uma rajada de vento verdadeira de 2° durando meio
   segundo saía do filtro com menos da metade da amplitude.
2. **Os apps só enxergam 4-5 Hz.** O poll Modbus roda a 250 ms e o notify BLE a
   200 ms, enquanto o sensor é amostrado a 100 Hz. Mesmo o que sobrava do pico
   caía entre duas leituras.

Medido em simulação da cadeia completa, percentual do pico de uma rajada real
de 2,0° que chegava ao mín/máx:

| duração da rajada | 0,3s | 0,5s | 1,0s | 2,0s |
|---|---|---|---|---|
| como era (valor da tela, a 4-5 Hz) | 24% | 39% | 64% | 80% |
| **como é agora (100 Hz no firmware)** | **76%** | **88%** | **96%** | **99%** |

Para uma medição de carga de vento, esse é o erro que mais importa: ele é
sistemático e sempre **para menos**, ou seja, o relatório subestimava o
esforço real na estrutura.

### Como funciona

A mesma amostra de 100 Hz passa a alimentar **três caminhos independentes**,
cada um com o filtro que a sua finalidade pede:

| caminho | filtro | destino |
|---|---|---|
| **Tela** | 1-euro adaptativo (v1.4.0) | display + degrau de 0,25° |
| **Medida** | 1 polo em `ANGLE_PEAK_CUTOFF_HZ` (3 Hz) + peak-hold | mín/máx, histórico, relatório |
| **Vibração** | nenhum (amostra crua) | FFT do Modo Vibração |

O corte de 3 Hz cobre a banda física do balanço do mastro (1-5 Hz) sem deixar
passar o ruído de alta frequência do acelerômetro. Sozinho, porém, ele deixa
o ruído restante virar extremo falso — daí a segunda peça, o **teste de
persistência** (`ANGLE_PEAK_PERSIST_SAMPLES`, 10 amostras = 100 ms): um
extremo só é aceito se o sinal **se manteve** lá pela janela inteira. Ruído
não se mantém; um movimento real do mastro se mantém.

Com o eixo parado por 10 minutos, faixa mín/máx falsa (só ruído):

| | 1x ruído | 2x | 3x |
|---|---|---|---|
| filtro de 3 Hz sozinho | 0,340° | 0,620° | 0,910° |
| **+ persistência de 100 ms** | **0,240°** | 0,490° | 0,750° |
| (referência: como era antes) | 0,120° | 0,230° | 0,330° |

No ruído nominal a faixa falsa fica abaixo de um degrau de tela — em troca de
capturar 88% de uma rajada de meio segundo em vez de 39%.

### O eixo de pan é diferente

No pan o peak-hold é **simples**: sem filtro extra e sem teste de
persistência. Dois motivos:

- **Não há ruído a filtrar.** O ângulo de pan não vem do acelerômetro, vem de
  uma integração cujo cancelamento de janela parada (ver "Azimute (pan)"
  acima) já deixa o valor cravado com o eixo parado.
- **Persistência custaria caro ali.** O motor gira a 20-30°/s, então 100 ms de
  exigência cortariam 2-3° do ponto de retorno de cada varredura — justamente
  o extremo que se quer registrar.

O que o peak-hold resolve no pan é só o segundo problema da lista lá em cima:
a 4-5 Hz de polling, uma varredura rápida passa pelo extremo entre duas
leituras do app e some.

### Reset e calibração

Calibrar zera os extremos junto, porque eles são relativos ao zero antigo e
deixariam de significar qualquer coisa. Para esquecer só os extremos, sem
mexer no zero, existe uma ação separada (coil 2 / characteristic
`6e6e000b-...`) — é o que o botão "Resetar limites" dos apps usa.

### Compatibilidade

Aditiva, como as anteriores. Um app que não conhece os registradores/a
characteristic nova simplesmente não os lê e volta a calcular os extremos por
conta própria; um firmware antigo responde com exceção de endereço inválido e
os apps encolhem o pedido automaticamente.

## Limitações conhecidas / próximos passos

- **[1.5.0] O peak-hold não foi confirmado em hardware.** Os números acima
  vêm de simulação da cadeia, e a classe `PeakHold` real foi exercitada
  contra essa mesma simulação (reproduz os valores da tabela). O ajuste, se
  no ESP32 real a faixa falsa ficar grande demais: subir
  `ANGLE_PEAK_PERSIST_SAMPLES` (mais rejeição de ruído, menos captura de
  rajada curta) ou baixar `ANGLE_PEAK_CUTOFF_HZ`. Na direção oposta — se o
  mín/máx estiver perdendo eventos reais curtos — baixar a persistência.

- **[1.5.0] O gráfico do relatório continua sendo o valor da tela.** Só o
  mín/máx do resumo (e os marcadores de extremo) vêm do peak-hold. Por isso
  um marcador pode aparecer *fora* da curva: a curva mostra o que a tela
  mostrou, o marcador mostra o que de fato aconteceu. Traçar a curva do
  caminho de medida exigiria transmitir uma segunda série contínua, o que não
  parece valer o custo de banda — o valor que importa para o laudo é o
  extremo.

## Taxa de amostragem alta no Modo Vibração (v1.4.0)

A captura passou a aceitar até `VIBRATION_MAX_RATE_HZ` = **500 amostras/s**
(antes o app limitava em 200). Três coisas tiveram que mudar juntas para
que essa taxa medisse algo de verdade:

1. **Temporização em microssegundos.** O período era `1000/rateHz` em
   milissegundos, truncado por divisão inteira: 300Hz viravam 3ms, ou seja,
   333Hz de verdade. Como os apps calculam todas as frequências do espectro a
   partir da taxa *pedida*, esse desvio deslocava o espectro inteiro. Em
   microssegundos o erro fica abaixo de 0,1% em toda a faixa. O instante-alvo
   também passou a avançar pelo período exato, em vez de reancorar em "agora"
   a cada amostra — assim o atraso de uma amostra não se acumula nas
   seguintes.
2. **I2C a 400kHz** (`Wire.setClock`). Cada amostra faz duas transações de 14
   bytes (tilt e pan), que a 100kHz custam ~2,8ms somadas — mais que o
   período de 2ms de uma captura a 500Hz. A 400kHz o mesmo par custa ~0,7ms.
3. **Banda do DLPF conforme a taxa.** O filtro interno do MPU6050 ficava
   cravado em 21Hz, o que é ótimo para a leitura contínua mas tornaria os
   500Hz inúteis: o sinal sairia do chip já filtrado a 21Hz e a taxa maior
   não mediria nada de novo. Agora a banda é aberta no início da captura
   conforme a taxa (94Hz a partir de 400 amostras/s, 44Hz a partir de 200) e
   volta para 21Hz ao final, mantendo folga para o filtro seguir servindo de
   anti-aliasing.

O teto de `VIBRATION_MAX_SAMPLES` continua em 6000 amostras por eixo (limite
de RAM), então a 500Hz a captura é truncada em ~12s — o app avisa isso na
tela de configuração, em vez de truncar em silêncio.

## Limitações conhecidas / próximos passos

- **[1.5.0] O peak-hold não foi confirmado em hardware.** Os números acima
  vêm de simulação da cadeia, e a classe `PeakHold` real foi exercitada
  contra essa mesma simulação (reproduz os valores da tabela). O ajuste, se
  no ESP32 real a faixa falsa ficar grande demais: subir
  `ANGLE_PEAK_PERSIST_SAMPLES` (mais rejeição de ruído, menos captura de
  rajada curta) ou baixar `ANGLE_PEAK_CUTOFF_HZ`. Na direção oposta — se o
  mín/máx estiver perdendo eventos reais curtos — baixar a persistência.

- **[1.5.0] O gráfico do relatório continua sendo o valor da tela.** Só o
  mín/máx do resumo (e os marcadores de extremo) vêm do peak-hold. Por isso
  um marcador pode aparecer *fora* da curva: a curva mostra o que a tela
  mostrou, o marcador mostra o que de fato aconteceu. Traçar a curva do
  caminho de medida exigiria transmitir uma segunda série contínua, o que não
  parece valer o custo de banda — o valor que importa para o laudo é o
  extremo.

- **[1.4.0] O filtro adaptativo não foi confirmado em hardware.** Os números
  acima vêm de simulação da cadeia, e o código real do `AngleSensor.cpp` foi
  conferido contra essa simulação (bate até 8e-6°, só arredondamento de
  `float`). Mas ruído real inclui vibração mecânica, montagem e temperatura,
  que a simulação não modela. Se a leitura tremular no ESP32 real, o primeiro
  ajuste é baixar `ANGLE_FILTER_MIN_CUTOFF_HZ`; se ficar lenta demais para
  acompanhar o movimento, subir `ANGLE_FILTER_BETA`.

- **[1.4.0]** O Modo Vibração via BLE entregava séries corrompidas em
  silêncio. Notificação BLE não tem confirmação: com a fila do rádio cheia, o
  pacote some sem aviso, e o app montava a série ignorando o buraco — o que
  adiantava no tempo todas as amostras seguintes e falseava o espectro
  inteiro. Agora o app detecta o buraco e pede retransmissão pela
  characteristic `6e6e000a-...`; se ainda assim não completar, reporta erro em
  vez de entregar um resultado plausível e errado. O intervalo entre pacotes
  caiu de 20ms para 5ms (400 → 1600 amostras/s), o que só é seguro porque a
  perda agora é recuperável.

- **[1.4.0]** `VibrationCapture::start()` era chamado de dentro do callback de
  escrita do BLE, que roda na task do stack BLE, enquanto `update()` mexe nos
  mesmos campos no loop principal. Agora o callback só agenda
  (`requestStart()`) e quem inicia é o loop — mesmo padrão já usado para
  reiniciar o anúncio BLE.

- **[1.3.1]** O ESP32 não voltava a anunciar por BLE depois que um cliente
  desconectava. O rádio para de anunciar sozinho ao aceitar uma conexão, e a
  lib BLE do Arduino não retoma o anúncio na desconexão — resultado: o
  dispositivo sumia do scan até ser resetado na mão. Aparecia ao trocar de
  app, ou só ao reabrir o mesmo app, e parecia defeito de hardware. Agora
  `BleServer` registra um `BLEServerCallbacks` que marca a desconexão, e o
  anúncio é reiniciado no `loop()` — e não dentro do próprio callback, que
  roda na task do stack BLE, onde essa chamada tende a falhar em silêncio.

- **[1.2.0] Faixa do pan é placeholder.** `PAN_MIN_DEG`/`PAN_MAX_DEG` estão
  em ±90° até a mecânica do pan estar definida. Só limita o valor
  reportado — o integrador interno não é clampado, então voltar para dentro
  da faixa recupera a leitura correta.
- **[corrigido parcialmente na 1.6.9] `PAN_SCALE_CORRECTION` calibrado com
  só 2 pontos.** Ver "Azimute (pan) pelo giroscópio" acima — reduz o erro de
  ±0,25° a 8-9° para ±0,02° nesses dois pontos, mas com uma referência de
  precisão não quantificada e só 2 pontos (o tilt teve 24, contra um
  inclinômetro certificado). Refazer com mais pontos, mais afastados de
  zero, quando possível.
- **[1.2.0 → corrigido na 1.6.15] Sinal do termo de compensação de tilt.**
  Estava trocado (`−gy·sin θ`): a taxa de pan saía multiplicada por
  `cos(2θ)`. Não depende da montagem — o acelerômetro e o giro do mesmo chip
  dão a vertical e a velocidade angular nos mesmos eixos, e a projeção é o
  produto escalar dos dois. Confirmar em bancada mesmo assim: o mesmo giro
  de pan com o tilt em 0° e em ±30°/±45° tem de dar o mesmo ângulo.
- **[1.2.0 → corrigido na 1.6.17] Pan supunha o eixo X do chip na
  horizontal.** A projeção usava só `gy`/`gz`; com o X a φ da horizontal a
  taxa saía × cos φ (bancada: 40 → 28,75 = 40·cos 44°). Agora são os três
  eixos. Conferir com `python-app/tools/diagnostico_pan.py`, coluna
  "X fora": o ângulo do eixo X fora da horizontal.
- **[1.6.17 → desfeito na 1.6.18] Sentido do tilt.** A 1.6.17 passou
  `TILT_DIRECTION_SIGN` para −1 (na bancada com a 1.6.16 subir lia
  negativo), mas com a 1.6.17 o tilt positivo passou a aparecer negativo:
  voltou a +1. Só o ângulo reportado — os lados da correção de escala seguem
  o sinal do ângulo bruto.
- **[1.6.15 → corrigido na 1.6.16] Sentido do pan invertido.** Com o sinal
  corrigido, a 1.6.15 lia positivo no sentido anti-horário visto de cima, e
  o pan do usuário é positivo no horário: `PAN_DIRECTION_SIGN = −1`. A 1.6.14
  acertava o sentido por acaso — o sinal trocado de `gy` invertia o sentido
  de novo porque o sensor está montado a mais de 45° da posição deitada
  (`atan2(ay, az)` entre 45° e 135°). Conferir em bancada: giro horário visto
  de cima tem de ler positivo.
- **[corrigido na 1.6.6] Modo Vibração via USB com uma amostra errada a cada
  32.** O bloco de amostras de tilt ocupa os input registers 31-62, que
  incluem o 40 — o da versão do firmware. Como a versão era testada antes,
  a 10ª amostra de todo bloco saía como 10605 (106,05°): em teste, 3 Hz e
  0,5° viravam 3,12 Hz e 7,06°. Agora um bloco só vale para leituras que
  COMEÇAM no início dele (31 ou 70), que é como o app lê; a versão
  continua em 40, lida sozinha. Via BLE não era afetado.
- **[1.6.6] Ressincronização do Modbus**: lixo colado antes de um pedido
  (ruído ao abrir a porta) invalidava o CRC e o pedido se perdia; como todo
  pedido atendido tem 8 bytes, os 8 finais são aproveitados se válidos.
- **Teste de ponta a ponta do modo USB** (`sim/e2e_modbus.py` +
  `sim/modbus_sim.cpp`, parte do `run.sh`): firmware real em tempo real numa
  porta serial virtual, contra o código real do app com o pymodbus 3.14,
  com a placa reiniciando a cada abertura da porta, reset no meio da
  leitura e linha com bytes perdidos/corrompidos. Mudanças do lado do app
  (`python-app/data_source/modbus_source.py`): porta aberta com DTR/RTS
  desligados (evita o auto-reset da placa, que apagava a calibração a cada
  "Testar conexão"/"Iniciar"/reconexão — a confirmar em bancada, depende do
  driver do chip USB-serial), sondagem até a placa responder em vez de
  espera fixa, timeout de 0,3 s com 1 repetição (antes cada falha isolada
  congelava a leitura por 4 s), reconexão automática do pymodbus 3.x
  desligada (ela reabria a porta durante o boot e podia reiniciar a placa
  em ciclo), teste de conexão tolerante e fora da thread da UI, captura de
  vibração com repetição por transação, e checagem da versão do pymodbus
  (a API `device_id` exige 3.10+).
- **[corrigido na 1.6.5] Robustez contra falhas de hardware**, encontradas
  pelo teste de resistência (`sim/soak_sim.cpp`):
  - **leitura I2C que falhava durante um giro jogava fora 10 ms de rotação**
    (o relógio da integração avançava mesmo sem amostra) — 0,2° perdidos
    por falha, em silêncio;
  - **sensor reiniciado nunca era reconfigurado**: numa queda de
    alimentação o MPU6050 volta em SLEEP, lendo zeros, até o próximo boot
    do ESP32. `Mpu6050::maintain()` relê a configuração a cada 1 s e
    reconfigura; com 20 leituras seguidas falhando, reinicia também o
    periférico I2C;
  - **quadros de lixo** (bytes 0xFF, zeros, aceleração com módulo fora de
    0,5–2 g, giro no fundo de escala) são descartados no driver;
  - **leitura travada no limite**: o integrador agora é limitado à faixa
    (anti-windup) — passar de ±90° e voltar responde na hora, em vez de só
    descontar do excesso;
  - **tilt**: mediana de 5 amostras antes do filtro 1-euro, que seguia um
    pico isolado e levava segundos para esquecê-lo (e o gravava nos
    extremos); o tilt da projeção do pan também passa pela mediana;
  - **depois de um buraco** (loop atrasado, leituras perdidas) o histórico
    da mediana é descartado e o buraco só é integrado com 5 amostras novas;
  - **precisão**: integração por trapézio (erro de até 0,2° por borda de
    giro), janela só é "parada" se nenhuma amostra passar de 8°/s (bordas
    de giro eram canceladas como ruído), e bias com estimativa da
    velocidade de deriva térmica (filtro alfa-beta) extrapolada durante os
    giros.
- **[1.2.0 → corrigido na 1.6.4] Pan da placa PARADA derivava até cravar
  em ±90° depois de alguns minutos.** Uma única leitura errada do giroscópio
  no barramento I2C (ex.: −250°/s por 10 ms) tirava a janela de ZUPT da
  condição de "parada" e os ~2,5° integrados nela ficavam para sempre.
  Algumas por minuto bastavam: em simulação, uma a cada ~7 s leva a −84° em
  4 min e crava em −90° (cenário I de `sim/pan_sim.cpp`). A 1.6.4 passa gy/gz
  por uma **mediana móvel de 5 amostras** antes de integrar (~20 ms de
  atraso) — cenários I e J cravados no valor certo por 10 min. O contador
  `espurias` do diagnóstico BLE mostra quantas leituras foram descartadas.
- **[1.2.0 → corrigido na 1.6.2] Boot com a placa em movimento travava o
  pan em ±90°.** Até a 1.6.1 a primeira janela de ZUPT era aceita sem teste
  como bias. Ligada em movimento, o bias saía errado, toda janela parada
  seguinte era rejeitada e a leitura derivava até cravar em ±90° **para
  sempre** (só Calibrar recuperava). Aparecia no uso via BLE (app Android,
  placa ligada na mão) e não via USB, porque o app Python reinicia o ESP32
  ao abrir a porta serial, com a placa parada na bancada. A 1.6.2 só adota
  o bias quando **duas janelas seguidas têm médias coerentes** e, como rede
  de segurança, reaprende o bias (desfazendo a deriva) após
  `PAN_BIAS_RELEARN_WINDOWS` (20) janelas coerentes entre si mas longe dele
  — ver "BOOT E RECUPERAÇÃO" em `PanSensor.h`. Cenários F e G de
  `sim/pan_sim.cpp`: na 1.6.1, −90° travado; na 1.6.2, 29,75° e 0,00°
  corretos. O cenário H confirma que varreduras do motor (8 s a 20°/s) não
  disparam o reaprendizado.
- **[1.2.0 → reduzido na 1.6.14] Movimentos pequenos são descartados** como
  ruído, por causa do cancelamento de janela parada. Até a 1.6.13 o piso era
  ~1° — e um giro à mão lento era descartado inteiro (ver "Giro lento, feito
  à mão"); desde a 1.6.14 é ~0,1° para um movimento isolado, e as bordas
  suaves de um giro maior são preservadas. Ajustável em
  `PAN_ZUPT_RATE_THRESHOLD_DPS`/`PAN_ZUPT_WINDOW_MS`.
- **[1.2.0 → corrigido na 1.6.1] Mudança de tilt travava o pan em ±90°.**
  Até a 1.6.0 o bias era estimado sobre a taxa **já projetada**
  (`gz·cos θ + gy·sin θ`), que depende de `θ`. A premissa de que "a janela de
  ZUPT seguinte reabsorve a diferença" estava errada: com bias de fábrica
  típico, 45° de mudança de tilt deslocava o bias projetado em ~1,5°/s —
  acima do limiar de 1°/s —, então **toda** janela parada passava a ser
  rejeitada, o bias nunca mais se corrigia e o erro era integrado sem fim
  até cravar a leitura em ±90°, sem recuperar nem voltando a placa à posição
  original. Relatado em bancada ("mudo a posição da placa e o pan trava").
  A 1.6.1 estima o bias **por eixo, no referencial do sensor** (`gy`, `gz`),
  e só projeta depois — ver o item 2 do cabeçalho de `PanSensor.h`.
- **[corrigido na 1.6.1] Modo Vibração e versão do firmware sumiam no BLE.**
  `createService(SERVICE_UUID)` reserva só 15 handles GATT, e o serviço usa
  29 (2 por characteristic + 1 por descritor BLE2902). A lib não dá erro:
  deixa de registrar, em silêncio, o que não coube — as characteristics de
  configuração/status/dados/retransmissão da vibração e a da versão. Os
  apps viam "Characteristic 6e6e0005-... was not found!" ao iniciar a
  captura e "firmware v?" no teste de conexão. O serviço agora é criado com
  `SERVICE_NUM_HANDLES` explícito (`BleServer.cpp`).
- **[corrigido na 1.6.1] Pedido de retransmissão de um eixo podia sobrescrever
  o do outro.** O app pede tilt e pan na mesma rodada, e havia uma vaga só
  para o pedido pendente. Agora é uma vaga por eixo, com consumo atômico.
  Medido em simulação com 30% de pacotes perdidos: antes, 1464 amostras
  faltando após as retransmissões; depois, captura completa.
- **[1.3.0] Memória do buffer de captura dobrou** para ~24KB (dois buffers
  de `VIBRATION_MAX_SAMPLES` int16, em `VibrationCapture.h`). Cabe com folga
  no ESP32 mesmo com o stack BLE ativo, mas é o maior consumo de RAM do
  firmware — é aqui que o limite aperta se um dia precisar de capturas mais
  longas.
- **[1.3.0] Carga do barramento I2C subiu**: durante uma captura de vibração
  são duas transações por amostra (tilt e pan), somadas às do loop contínuo.
  Na taxa padrão de 50Hz a ocupação fica em torno de 40% a 100kHz, com folga
  confortável. Em taxas de amostragem bem mais altas isso pode apertar — a
  saída é subir o barramento para 400kHz (`Wire.setClock`), que o MPU6050
  suporta. Não foi feito de propósito: mexeria numa temporização já validada
  em hardware sem necessidade comprovada.
- **[1.2.0] Os dois apps já consomem o pan** (Modbus reg. 1 e BLE
  `6e6e0008-...`), com segundo display, segundo par de mín/máx e segundo
  gráfico no relatório. Ambos detectam sozinhos um ESP32 com firmware
  anterior à v1.2.0 e seguem só com a inclinação, sem gravar zeros falsos.
- **[1.0.2]** Faixa de medição mudou de 0°~120° para -60°~+60° (0° agora é
  a posição calibrada, não mais um extremo mecânico). Isso muda a
  codificação do ângulo em `REG_ANGLE_INPUT` (Modbus) e na characteristic
  de ângulo (BLE): antes era sempre não-negativo, agora é `int16` com
  sinal — os dois apps (`modbus_source.py`/`ble_source.py`,
  `BleAngleDataSource.kt`) já foram atualizados pra decodificar como
  valor com sinal. Firmware em si não mudou nada na codificação (o cast
  pra `uint16_t` já produzia o padrão de bits correto em complemento de
  dois para valores negativos) — só os limites `ANGLE_MIN_DEG`/
  `ANGLE_MAX_DEG` em `Config.h`.
- **[1.0.1]** `BleServer::updateVibrationNotify()` notificava o status
  "pronto" da captura de vibração *antes* de terminar de transmitir todas
  as amostras pela characteristic de dados, e chamava `notify()` sem
  nenhum controle de taxa (a cada iteração do `loop()`) — o app Python
  (`ble_source.py`) libera o resultado assim que recebe o "pronto", então
  isso causava dados truncados/perdidos no Modo Vibração via BLE, além do
  risco de sobrecarregar a fila de notificação do BLE numa captura longa.
  Corrigido: agora os dados são enviados primeiro (com um intervalo mínimo
  de `BLE_VIBRATION_CHUNK_INTERVAL_MS` entre pacotes) e o "pronto" só é
  notificado depois que o último bloco foi enviado; o status/progresso
  durante a captura também passou a ser limitado por
  `BLE_VIBRATION_STATUS_NOTIFY_INTERVAL_MS`. Ainda não validado contra
  hardware real (só revisão de código) — atenção especial a isso ao testar
  o Modo Vibração via BLE.
- Buffer de captura de vibração limitado a `VIBRATION_MAX_SAMPLES = 6000`
  amostras (~12KB de RAM) — captura pedida além disso é truncada
  silenciosamente (menos amostras que o solicitado, mas sem erro).
- Calibração não é persistida entre reboots (offset fica só em RAM).
- Framing Modbus RTU usa um timeout fixo simplificado (5ms de silêncio)
  para detectar fim de frame, em vez do cálculo exato de 3.5 caracteres
  do padrão — funciona bem na baudrate usada no projeto (9600), mas pode
  precisar de ajuste fino em baudrates mais altas.
- A UART0 (porta USB) fica dedicada ao protocolo Modbus RTU — não há canal
  de log/debug separado disponível hoje (ver comentário em `main.cpp`).
- Orientação de montagem do sensor tem recomendação definida (eixo X no
  giro, plano Y-Z), mas confirmação física em bancada ainda pendente,
  como descrito acima (a pinagem em si já está definida).
- Para o cabo até o painel de controle (~7m), use um **cabo de extensão
  USB ativo** — USB simples sem amplificação só é confiável até uns 5m.
