# Pinout — Inclinômetro ESP32

Mapeamento completo de pinos do ESP32, para o hardware **definido e
confirmado**: **MPU6050 via I2C** (sensor) + **comunicação com o PC via
cabo USB direto** (sem RS485 — decisão tomada explicitamente, ver
`firmware/README.md`). Placa confirmada: **ESP32-DevKitC V4** (placa
oficial da Espressif, módulo WROOM-32), com chip conversor USB-serial
**CP2102N** (Silicon Labs) integrado à placa.

## I2C — MPU6050 (acelerômetro)

| Sinal | Pino ESP32 | Vai para | Observação |
|---|---|---|---|
| SDA | **GPIO 21** | SDA do MPU6050 | Pino I2C padrão do ESP32 (`Wire.begin()` sem argumentos) |
| SCL | **GPIO 22** | SCL do MPU6050 | Pino I2C padrão do ESP32 |
| Alimentação | **3.3V** | VCC do MPU6050 | **Não usar 5V** — a maioria dos módulos GY-521 tem pull-up de SDA/SCL para o VCC; em 5V, as linhas I2C sobem a 5V e podem danificar as entradas do ESP32 |
| Terra | **GND** | GND do MPU6050 | Referência comum |
| Endereço | **GND** | AD0 do MPU6050 | Fixa o endereço I2C em `0x68`, o que o driver (`firmware/src/Mpu6050.h`) assume |
| Interrupção | *(não conectado)* | INT do MPU6050 | Não usado hoje (leitura por polling); ver "Próximos passos" |

**Os dois eixos medidos saem deste mesmo módulo, sem fiação adicional.** A
inclinação (*tilt*) vem do acelerômetro; o azimute (*pan*) vem do
**giroscópio** do mesmo MPU6050, integrado com ZUPT (ver "Azimute (pan) pelo
giroscópio" em `firmware/README.md`). Do ponto de vista elétrico não mudou
nada em relação à v1.1.0 — o firmware só passou a ler 14 bytes em vez de 6 no
burst I2C, trazendo acelerômetro, temperatura e giroscópio de uma vez. Foi
justamente por isso que a alternativa de um encoder rotativo dedicado
(AS5600, que exigiria módulo novo e um ímã diametral montado no eixo) acabou
descartada.

## USB — comunicação com o PC

Não usa nenhum GPIO adicional: a comunicação Modbus RTU trafega pela **porta
USB nativa do ESP32** (o mesmo cabo/porta usado para gravar o firmware),
internamente ligada à UART0 do chip.

| Item | Valor | Observação |
|---|---|---|
| Cabo | USB padrão (A-MicroUSB ou A-C, conforme a placa) | Sem hardware adicional |
| Alcance sem amplificação | ~5m | Limite do USB 2.0 padrão |
| Alcance necessário no projeto | ~7m (mastro do pan-tilt) | **Excede o limite padrão** |
| Solução | **Cabo de extensão USB ativo** (10-20m, com amplificador embutido) | Vendido pronto, sem precisar de hub/alimentação extra no meio do caminho |

### Chip conversor USB-serial (na placa, não no ESP32)

O ESP32 (módulo WROOM-32) **não tem USB nativo** — quem faz a conversão
USB ↔ UART é um chip à parte, já embutido na placa DevKitC. Na placa usada
neste projeto, esse chip é o **CP2102N** (fabricante Silicon Labs, mesma
família dos CP210x, consagrada em placas de desenvolvimento). Isso já
resolve tudo sozinho:

- Não precisa de nenhum adaptador externo — o CP2102N já faz esse papel,
  sem componentes USB adicionais (USB 2.0 full-speed, até 3 Mbaud).
- Normalmente é reconhecido automaticamente pelo Windows 10/11 (driver
  Silicon Labs CP210x costuma já vir no sistema) e nativamente no Linux.
  Se a porta COM não aparecer, instalar o driver **CP210x VCP** da
  Silicon Labs (ver `python-app/windows/INSTALACAO_WINDOWS.md`).
- Alimentação do CP2102N vem do **VBUS (5V)** do próprio USB, com
  regulação interna para a lógica em 3,3V — o 3,3V do ESP32/MPU6050 vem de
  um regulador separado, já embutido na placa DevKitC.
- VID:PID de fábrica bem conhecido e estável (`10C4:EA60`, Silicon Labs) —
  diferente do chip anterior (CH9102X), cujo VID/PID não era confiável o
  bastante para filtrar portas por ele (ver nota em
  `python-app/data_source/modbus_source.py`).

## Bluetooth LE

Não usa GPIO — o rádio BLE é interno ao módulo ESP32 (antena embutida na
maioria das placas DevKit). Nenhuma consideração de fiação adicional.

## Referência completa de pinos — datasheet ESP32-WROOM-32 (Espressif)

Tabela de pinos do módulo, com os **nomes originais do fabricante**
(coluna "Nome", exatamente como aparece no *ESP32-WROOM-32 Datasheet* da
Espressif — Table 4, "Pin Definitions"), o número do pino no encapsulamento
(1-38), o alias GPIO (o que o código/Arduino usa) e o status neste projeto.

Nem toda placa DevKit expõe os 38 pinos do módulo (alguns fabricantes
omitem os pinos de flash e o NC) — mas os nomes abaixo são os do
datasheet do módulo, não de uma placa específica.

| Pino nº | Nome (datasheet) | GPIO | Status / uso neste projeto |
|---|---|---|---|
| 1 | **GND** | — | Terra |
| 2 | **3V3** | — | Alimentação 3.3V → VCC do MPU6050 |
| 3 | **EN** | — | Reset/enable do chip (botão RESET em muitas placas) |
| 4 | **SENSOR_VP** | GPIO36 | Só entrada (sem pull-up/down interno) |
| 5 | **SENSOR_VN** | GPIO39 | Só entrada (sem pull-up/down interno) |
| 6 | **IO34** | GPIO34 | Só entrada (sem pull-up/down interno) |
| 7 | **IO35** | GPIO35 | Só entrada (sem pull-up/down interno) |
| 8 | **IO32** | GPIO32 | Livre — também ADC1 |
| 9 | **IO33** | GPIO33 | Livre — também ADC1 |
| 10 | **IO25** | GPIO25 | Livre — também DAC1 |
| 11 | **IO26** | GPIO26 | Livre — também DAC2 |
| 12 | **IO27** | GPIO27 | Livre |
| 13 | **IO14** | GPIO14 | Livre |
| 14 | **IO12** | GPIO12 | ⚠️ Strapping — seleciona a tensão da flash, cuidado extra |
| 15 | **GND** | — | Terra |
| 16 | **IO13** | GPIO13 | Livre |
| 17 | **SHD/SD2** | GPIO9 | 🚫 Reservado (flash interna) — nunca usar |
| 18 | **SWP/SD3** | GPIO10 | 🚫 Reservado (flash interna) — nunca usar |
| 19 | **SCS/CMD** | GPIO11 | 🚫 Reservado (flash interna) — nunca usar |
| 20 | **SCK/CLK** | GPIO6 | 🚫 Reservado (flash interna) — nunca usar |
| 21 | **SDO/SD0** | GPIO7 | 🚫 Reservado (flash interna) — nunca usar |
| 22 | **SDI/SD1** | GPIO8 | 🚫 Reservado (flash interna) — nunca usar |
| 23 | **IO15** | GPIO15 | ⚠️ Strapping — verbosidade do log de boot |
| 24 | **IO2** | GPIO2 | ⚠️ Strapping — ligado ao LED onboard em muitas placas |
| 25 | **IO0** | GPIO0 | ⚠️ Strapping — botão BOOT em muitas placas |
| 26 | **IO4** | GPIO4 | Livre — antes era o controle de direção do RS485 (removido) |
| 27 | **IO16** | GPIO16 | Livre — antes era RX do RS485/UART2 (removido) |
| 28 | **IO17** | GPIO17 | Livre — antes era TX do RS485/UART2 (removido) |
| 29 | **IO5** | GPIO5 | ⚠️ Strapping — seleciona modo de boot SPI |
| 30 | **IO18** | GPIO18 | Livre |
| 31 | **IO19** | GPIO19 | Livre |
| 32 | **NC** | — | Não conectado |
| 33 | **IO21** | GPIO21 | ✅ **Em uso — SDA** (I2C, MPU6050) |
| 34 | **RXD0** | GPIO3 | ✅ **Em uso — UART0 RX** (Modbus RTU via USB) |
| 35 | **TXD0** | GPIO1 | ✅ **Em uso — UART0 TX** (Modbus RTU via USB) |
| 36 | **IO22** | GPIO22 | ✅ **Em uso — SCL** (I2C, MPU6050) |
| 37 | **IO23** | GPIO23 | Livre |
| 38 | **GND** | — | Terra |

## Notas

- **Orientação de montagem do sensor**: a fórmula do ângulo em
  `firmware/src/AngleSensor.cpp` (`atan2(ay, az)`) assume uma orientação
  específica dos eixos do MPU6050 — **recomendação definida, confirmação
  física ainda pendente**:
  - Montar o MPU6050 com o **eixo X alinhado ao eixo mecânico de giro do
    pan-tilt** (o pino/articulação de elevação), de forma que a rotação de
    -60° a +60° (120° de curso total) aconteça no **plano Y-Z** do sensor —
    é exatamente o par de eixos que `atan2(ay, az)` usa.
  - Motivo: com dois eixos (`atan2`) a sensibilidade da leitura fica quase
    constante em toda a faixa, sem "zona morta" perto de 90° — diferente
    de usar um único eixo (`asin`/`acos`), que perde resolução exatamente
    ali. Isso é crítico não só para a leitura de ângulo contínua, mas
    principalmente para o **Modo Vibração**, que precisa detectar
    variações de frações de grau em torno do zero calibrado (90°) — é
    justo a região onde um único eixo teria a pior sensibilidade.
  - Acelerômetro puro (sem giroscópio) não distingue inclinação angular de
    aceleração linear/translacional, mas vento em um pan-tilt no topo de
    um mastro tende a produzir majoritariamente flexão angular (o mastro
    dobra, a ponta gira) — exatamente o que se quer medir — então essa
    abordagem é adequada sem precisar de fusão com giroscópio.
  - **Confirmação em bancada** (pendente, ao montar o sensor fisicamente):
    gravar `ax, ay, az` brutos enquanto o eixo é movimentado manualmente de
    um extremo ao outro do curso (-60° a +60°) — confirma visualmente qual
    par de eixos varre o arco (um deles deve manter módulo baixo e quase
    constante: esse é o X, o eixo
    de giro) e os sinais corretos para o `atan2()` bater com o sentido
    físico (crescente = subindo).
- **Orientação do giroscópio para o pan**: a compensação de tilt em
  `firmware/src/PanSensor.cpp` (`ω_pan = gz·cos θ − gy·sin θ`) assume a mesma
  orientação de montagem acima — eixo X no eixo mecânico de tilt, rotação no
  plano Y-Z. Duas coisas para confirmar na **mesma sessão de bancada**, já
  que dependem da mesma montagem:
  - **Sinal do termo `−gy·sin θ`**: depende da handedness real. Perto de
    `θ=0` esse termo some (sobra só `gz·cos θ`), então um sinal trocado não
    aparece com o tilt zerado. Medir o mesmo movimento de pan com o tilt em
    0° e depois em ±45°/±60°: os dois têm que dar o mesmo ângulo. Se o valor
    com tilt grande sair maior ou menor, é o sinal deste termo.
  - **Fator de escala** (`PAN_SCALE_CORRECTION`, calibrado preliminarmente
    em 1,030 — ver "Azimute (pan) pelo giroscópio" em `firmware/README.md`):
    girar o eixo de pan entre duas posições de separação angular conhecida
    e usar `(ângulo real / integrado)`. É o erro dominante depois que o
    ZUPT resolve o bias — tolerância de fábrica do giro é de ~±3%.
- **Variante de placa**: esta pinagem assume um ESP32-DevKitC V4 (módulo
  WROOM-32) — confirmado na placa física em uso, junto com o chip
  conversor USB-serial CP2102N (ver seção "USB" acima). Variantes
  diferentes (S3, C3, etc.) têm GPIOs restritos diferentes e podem
  precisar de ajuste.
- **Placa vs. módulo**: a tabela de referência usa os nomes do datasheet
  do **módulo** ESP32-WROOM-32 (Espressif). A serigrafia da sua placa
  DevKit específica pode rotular os pinos de forma simplificada (só o
  número do GPIO, ex: "21" em vez de "IO21") e pode não expor todos os 38
  pinos do módulo (os de flash e o NC costumam ficar de fora do conector).
- Fonte da verdade no código: `firmware/src/Config.h` (constantes de pino)
  e `firmware/README.md` (contexto e decisões).
