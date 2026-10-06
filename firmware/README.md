# Firmware — Inclinômetro ESP32

**Versão atual: `1.6.12`** (`firmware/src/Config.h`, `FIRMWARE_VERSION`) —
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

## Pinagem (ESP32 DevKit clássico / WROOM-32)

Hardware confirmado: placa **ESP32 DevKit clássico (WROOM-32)**, com chip
conversor USB-serial **CH9102X** (WCH) — mapeamento definido, refletido em
`firmware/src/Config.h`. Como a comunicação com o PC vai pela porta USB
nativa do ESP32, o único hardware extra é o MPU6050:

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

### Fator de escala (`TILT_SCALE_CORRECTION_NEG`/`_POS`, v1.6.8/1.6.10/1.6.11/1.6.12)

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

**Recalibrado outra vez na 1.6.12** com mais 15 pontos (0° a 8,03°): mesmo
com o fator `1,076`, ainda sobrava um viés de −0,05° em média (até −0,14°).
Juntando as três rodadas (9 + 15 + 15 = 39 pontos positivos, 0° a 21,5°,
tudo convertido para a base bruta do sensor), o fator medido é 0,9188.
Testado também um ajuste com deslocamento fixo além da escala — saiu em só
0,007°, confirmando que é escala pura, sem offset escondido.
`TILT_SCALE_CORRECTION_POS` passa para **1,088**: desvio-padrão do resíduo
cai de 0,101° para 0,084° e o viés médio zera. O lado negativo
(`TILT_SCALE_CORRECTION_NEG = 1,042`) segue com só os 9 pontos originais
(até −8,6°) — sujeito ao mesmo risco de ficar curto fora dessa faixa (como
o positivo já mostrou, duas vezes); mais pontos desse lado, numa faixa
maior, teriam o mesmo benefício. Refazer toda esta calibração se o MPU6050
físico desta unidade for substituído.

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
   `ω_pan = gz·cos(θ) − gy·sin(θ)`, com `θ` vindo do **mesmo burst I2C**.
   Por ser uma projeção (produto escalar), e não a fórmula de taxa de Euler
   `(gy·sinφ + gz·cosφ)/cosθ`, não há singularidade em nenhum tilt.
2. **ZUPT.** A cada 1s, se a **média** de `gy`/`gz` na janela estiver perto
   do bias corrente, a janela é dada como parada e o bias é refinado. Usar a
   média (e não o pico) deixa o detector imune a vibração, que é de média
   zero: o mastro pode estar balançando sob vento que a janela ainda é
   corretamente reconhecida como parada. O bias é estimado **por eixo, no
   referencial do sensor**, e só depois projetado — o bias de fábrica é fixo
   nos eixos do chip, e projetado ele mudaria com o tilt (ver a limitação
   corrigida na 1.6.1, mais abaixo).
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
preservados integralmente.

**Isso valida a matemática, não o hardware.** Falta rodar contra o MPU6050
real — ver as duas confirmações de bancada nas limitações abaixo.

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
e mantém a comparação entre as duas versões direta. Com esse degrau a tela já
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
- **[1.2.0] Sinal do termo de compensação de tilt a confirmar.** O termo
  `−gy·sin(θ)` depende da handedness real da montagem. Perto de `θ=0` ele
  some (só sobra `gz·cos θ`), então um sinal trocado **não apareceria** nos
  testes feitos com o tilt zerado — testar panning com o tilt em ±45°/±60° e
  conferir se bate com a mesma medida feita em `θ=0`. Resolve junto com a
  confirmação de montagem do `atan2(ay, az)`, que já estava pendente.
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
- **[1.2.0] Movimentos menores que ~1° são descartados** como ruído, por
  causa do cancelamento de janela parada. É o compromisso da abordagem:
  ajustável em `PAN_ZUPT_RATE_THRESHOLD_DPS`/`PAN_ZUPT_WINDOW_MS`, ao custo
  de uma estimativa de bias pior.
- **[1.2.0 → corrigido na 1.6.1] Mudança de tilt travava o pan em ±90°.**
  Até a 1.6.0 o bias era estimado sobre a taxa **já projetada**
  (`gz·cos θ − gy·sin θ`), que depende de `θ`. A premissa de que "a janela de
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
