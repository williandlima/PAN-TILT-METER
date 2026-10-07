# HARDWARE — Lista de Materiais (BOM)

Lista de materiais (*Bill of Materials*) do inclinômetro/azimutímetro ESP32,
com as informações técnicas principais de cada item e o link para o
datasheet do fabricante. Fonte da verdade para pinagem e decisões de
hardware: [`docs/pinout.md`](../docs/pinout.md) e
[`firmware/README.md`](../firmware/README.md) — esta lista resume e
referencia os mesmos componentes, sem duplicar o detalhamento elétrico.

Status de cada item: **confirmado** (hardware físico já definido e em uso
no projeto) ou **em avaliação** (opção considerada, ainda não decidida).

## 1. Lista de materiais confirmados

| # | Item | Qtde. | Especificações técnicas principais | Datasheet do fabricante | Observação |
|---|---|---|---|---|---|
| 1 | Placa de desenvolvimento oficial **ESP32-DevKitC V4** (Espressif, módulo **ESP32-WROOM-32**) | 1 | MCU Xtensa dual-core LX6 a até 240 MHz; Wi-Fi 802.11 b/g/n; Bluetooth Classic + BLE 4.2; 4 MB de flash (típico); 34 GPIOs; alimentação USB 5V com regulador 3,3V onboard | [ESP32-WROOM-32 Datasheet — Espressif Systems](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf) | Ambiente PlatformIO `esp32dev` ([`firmware/platformio.ini`](../firmware/platformio.ini)). Variantes diferentes (S3, C3 etc.) têm GPIOs restritos diferentes — ver nota em `docs/pinout.md`. |
| 2 | Chip conversor USB-serial **CP2102N** (Silicon Labs) — já embutido na placa DevKitC acima | 1 (integrado à placa) | Conversor USB 2.0 full-speed ↔ UART; até 3 Mbaud; alimentado pelo VBUS (5V) do próprio USB; VID:PID de fábrica estável (`10C4:EA60`) | [CP2102N Datasheet — Silicon Labs](https://www.silabs.com/documents/public/data-sheets/cp2102n-datasheet.pdf) | Não é um componente à parte a comprar — citado aqui porque o driver "CP210x VCP" da Silicon Labs pode precisar ser instalado manualmente no Windows, se não vier nativo (ver `python-app/windows/INSTALACAO_WINDOWS.md`). |
| 3 | Sensor inercial **MPU6050** (módulo breakout **GY-521**) | 1 | IMU de 6 eixos (acelerômetro + giroscópio, 3 eixos cada); saída digital de 16 bits; interface I²C; sem magnetômetro; endereço I²C fixado em `0x68` (AD0 → GND) | [MPU-6000/MPU-6050 Product Specification — InvenSense/TDK](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf) · [MPU-6000/6050 Register Map and Descriptions — InvenSense/TDK](https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf) | Alimentar em **3,3V** (não 5V — risco de dano ao ESP32, ver `docs/pinout.md`). Ligações: SDA→GPIO21, SCL→GPIO22, VCC→3V3, GND→GND, AD0→GND. |
| 4 | Cabo USB padrão (A–MicroUSB ou A–C, conforme o conector da placa) | 1 | USB 2.0; alcance confiável sem amplificação ≈ 5 m | — (item comercial genérico, sem datasheet específico) | Usado tanto para gravar o firmware quanto para o modo Real (USB/Modbus RTU) do software de supervisão. |
| 5 | Cabo de extensão USB **ativo** (com amplificador de sinal embutido), 10–20 m | 1 | Repetidor USB 2.0 ativo — necessário porque a distância até o painel de controle (≈ 7 m, ao longo do mastro) excede o alcance do USB simples | — (item comercial genérico; escolher modelo com amplificador ativo, não apenas cabo passivo) | Ver justificativa em `firmware/README.md`, seção de comunicação USB. |
| 6 | Regulador de tensão comutado ajustável **PTN78020W** (Texas Instruments) | 1 | Regulador *step-down* (buck) integrado; entrada larga 7–36 Vdc; saída ajustável 2,5–12,6V por resistor externo único; corrente de saída até 6A; eficiência de até 96% | [PTN78020W Datasheet — Texas Instruments](https://www.ti.com/lit/ds/symlink/ptn78020w.pdf) | **Resistor de ajuste de saída para 5V**: ligado entre os pinos 4 (VO Adjust) e 7 (GND), 1%, 0,05W, temperatura ≤100 ppm/°C — **21 kΩ, confirmado** na Tabela 3 (circuitos de aplicação típica) do datasheet (VI=15V, RSET=21kΩ → VO=5V, 88% de eficiência). Deriva o barramento de 5V do ESP32-DevKitC V4 a partir de uma fonte de tensão maior (bateria/fonte de campo — ver seção 2). |

## 2. Fonte de alimentação — opções em avaliação (não decidido)

O relatório técnico do projeto ([`docs/relatorio-tecnico-inclinometro.docx`](../docs/relatorio-tecnico-inclinometro.docx),
seção 4.5) traz um exemplo de dimensionamento de energia para operação em
campo, a **90 mA de consumo médio** (ESP32 com BLE ativo + sensor em
operação contínua), comparando fontes candidatas. A fonte primária (bateria)
ainda não foi definida como solução final — a lista abaixo é referência
para a decisão, não uma especificação de compra —, mas o regulador que
entrega os 5V ao ESP32 a partir dela **já está definido** (item 6 da
seção 1, PTN78020W).

| Fonte de alimentação | Autonomia estimada (90 mA médios) | Datasheet | Observação |
|---|---|---|---|
| Pilha alcalina 9V (ex.: Duracell MN1604) + regulador linear | ≈ 3,5 h | [MN1604 (6LR61) 9V Datasheet — Duracell](https://media.digikey.com/pdf/Data%20Sheets/Duracell/MN1604.pdf) | Grande parte da energia é dissipada como calor no regulador linear; alternativa ao PTN78020W, não usada em conjunto com ele |
| Pilha alcalina 9V (ex.: Duracell MN1604) + **PTN78020W** (item 6, seção 1) | ≈ 8 h | [MN1604 (6LR61) 9V Datasheet — Duracell](https://media.digikey.com/pdf/Data%20Sheets/Duracell/MN1604.pdf) | Reduz a corrente efetiva drenada da bateria em relação ao regulador linear |
| 4 pilhas AA (6V) + **PTN78020W** (item 6, seção 1) | ≈ 25 h | — (escolher modelo comercial de pilha) | Maior capacidade total em mAh; **atenção**: 6V está próximo do limite inferior de entrada do PTN78020W (mínimo de 7V, ou ≈2V acima da saída) — confirmar a tensão mínima das pilhas ao longo da descarga antes de adotar esta combinação |
| Bateria 18650 (3000 mAh) + regulador | ≈ 30 h | — (escolher modelo comercial) | Recarregável; tensão nominal (≈3,7V) exige regulador *boost* ou *boost-buck*, não o PTN78020W (que só reduz tensão) |
| Power bank USB (10.000 mAh) | ≈ 4 dias | — (escolher modelo comercial) | Já entrega 5V regulado pela própria USB; dispensa o PTN78020W nesta opção. Atenção ao corte automático em baixa corrente de alguns modelos |

## 3. Fora do escopo desta lista

A plataforma pan-tilt (motorizada, de inclinação e giro) no topo do mastro
é infraestrutura já existente da Avibras Aeroco, sobre a qual o hardware
acima é instalado — não é um item a especificar ou adquirir por este
projeto, e por isso não consta nesta lista de materiais.

## 4. Manutenção desta lista

Atualizar esta lista sempre que a pinagem (`docs/pinout.md`), as constantes
de hardware (`firmware/src/Config.h`) ou a decisão de fonte de alimentação
(seção 2) mudarem, para que ela continue refletindo o hardware
efetivamente usado no projeto.
