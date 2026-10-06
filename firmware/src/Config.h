#pragma once

#include <Arduino.h>

// ============================================================================
// Configuração de hardware — pinagem definida para ESP32 DevKit clássico
// (WROOM-32). Ver firmware/README.md para o mapeamento completo.
//
// Comunicação com o PC é via cabo USB direto (porta serial nativa do ESP32,
// a mesma usada para gravar o firmware) — não usa mais RS485, então não há
// pinos nem módulo transceptor adicionais para essa parte.
// ============================================================================

// I2C do MPU6050 (padrão do ESP32 DevKit). AD0 do sensor deve ir para GND
// (endereço 0x68, o que o driver em Mpu6050.h assume).
constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;

// Velocidade do barramento I2C. 400kHz (fast mode) em vez dos 100kHz padrão
// do Arduino — o MPU6050 suporta, e o Modo Vibração em taxa alta depende
// disso (ver Mpu6050::begin).
constexpr uint32_t I2C_CLOCK_HZ = 400000;

// Robustez do sensor (ver Mpu6050::maintain e accelPlausible). O chip pode
// reiniciar sozinho numa queda de alimentação — o rádio BLE puxa picos de
// corrente — e volta em SLEEP, lendo zeros, sem nunca mais ser
// reconfigurado. E o barramento pode devolver bytes corrompidos.
constexpr uint32_t MPU_HEALTH_CHECK_INTERVAL_MS = 1000;
constexpr uint16_t MPU_BUS_RESET_FAILURES = 20;
constexpr float ACCEL_MIN_PLAUSIBLE_G = 0.5f;
constexpr float ACCEL_MAX_PLAUSIBLE_G = 2.0f;

// ============================================================================
// Versão do firmware — bump manual a cada mudança relevante de contrato ou
// comportamento. FIRMWARE_VERSION_CODE codifica a mesma versão como inteiro
// (major*10000 + minor*100 + patch) para caber num único registrador
// Modbus/characteristic BLE de 16 bits (ex: "1.0.0" -> 10000).
// ============================================================================
constexpr char FIRMWARE_VERSION[] = "1.6.12";
constexpr uint16_t FIRMWARE_VERSION_CODE = 10612;

// ============================================================================
// Parâmetros Modbus RTU — devem bater com python-app/data_source/modbus_source.py
// ============================================================================
constexpr uint8_t MODBUS_SLAVE_ID = 1;
constexpr uint32_t MODBUS_BAUDRATE = 9600;

constexpr uint16_t REG_ANGLE_INPUT = 0;       // input register: ângulo de tilt * ANGLE_SCALE (int16 com sinal, faixa -60~+60°)
constexpr uint16_t REG_PAN_INPUT = 1;         // input register: ângulo de pan * ANGLE_SCALE (int16 com sinal, faixa PAN_MIN_DEG~PAN_MAX_DEG)
// Extremos medidos pelo peak-hold do firmware (ver PeakHold.h), na mesma
// codificação dos ângulos e relativos à mesma calibração. Ficam contíguos aos
// registradores 0 e 1 de propósito: o app lê os seis de uma vez só, numa
// única transação Modbus.
constexpr uint16_t REG_ANGLE_MIN_INPUT = 2;
constexpr uint16_t REG_ANGLE_MAX_INPUT = 3;
constexpr uint16_t REG_PAN_MIN_INPUT = 4;
constexpr uint16_t REG_PAN_MAX_INPUT = 5;
constexpr uint16_t COIL_CALIBRATE = 0;        // coil: write true -> zera os dois eixos (tilt e pan)
constexpr uint16_t COIL_VIBRATION_START = 1;  // coil: write true -> inicia captura de vibração
constexpr uint16_t COIL_RESET_PEAKS = 2;      // coil: write true -> esquece os extremos dos dois eixos
constexpr uint16_t REG_FIRMWARE_VERSION = 40; // input register: FIRMWARE_VERSION_CODE (somente leitura)

constexpr uint16_t REG_VIBRATION_DURATION = 10;  // holding register: duração da captura (s)
constexpr uint16_t REG_VIBRATION_RATE = 11;      // holding register: taxa de amostragem (Hz)

constexpr uint16_t REG_VIBRATION_STATUS = 20;       // input register: 0=ocioso,1=capturando,2=pronto,3=erro
constexpr uint16_t REG_VIBRATION_PROGRESS = 21;     // input register: percentual 0-100
constexpr uint16_t REG_VIBRATION_SAMPLE_COUNT = 22; // input register: total de amostras (quando pronto)

constexpr uint16_t REG_VIBRATION_CURSOR = 30;       // holding register: índice inicial do bloco a ler (vale para os dois eixos)
constexpr uint16_t REG_VIBRATION_BLOCK_START = 31;  // input register: início do bloco de amostras de TILT
constexpr uint16_t VIBRATION_BLOCK_SIZE = 32;       // amostras por bloco de leitura
constexpr uint16_t REG_VIBRATION_PAN_BLOCK_START = 70;  // input register: início do bloco de amostras de PAN (taxa * PAN_RATE_SCALE)

// ============================================================================
// Parâmetros BLE — devem bater com python-app/data_source/ble_source.py e
// android-app/.../datasource/BleContract.kt
// ============================================================================
constexpr char BLE_DEVICE_NAME[] = "Inclinometro-ESP32";
constexpr char SERVICE_UUID[] = "6e6e0001-3c17-4a2e-8f4b-1a2b3c4d5e6f";
constexpr char CHAR_ANGLE_UUID[] = "6e6e0002-3c17-4a2e-8f4b-1a2b3c4d5e6f";
constexpr char CHAR_CALIBRATE_UUID[] = "6e6e0003-3c17-4a2e-8f4b-1a2b3c4d5e6f";
constexpr char CHAR_VIBRATION_CONFIG_UUID[] = "6e6e0004-3c17-4a2e-8f4b-1a2b3c4d5e6f";
constexpr char CHAR_VIBRATION_STATUS_UUID[] = "6e6e0005-3c17-4a2e-8f4b-1a2b3c4d5e6f";
constexpr char CHAR_VIBRATION_DATA_UUID[] = "6e6e0006-3c17-4a2e-8f4b-1a2b3c4d5e6f";
constexpr char CHAR_FIRMWARE_VERSION_UUID[] = "6e6e0007-3c17-4a2e-8f4b-1a2b3c4d5e6f";
// Pan em characteristic própria, e não anexado à de ângulo, de propósito: os
// apps já instalados esperam exatamente 2 bytes em CHAR_ANGLE_UUID: estender
// aquela mensagem quebraria quem não foi atualizado. Assim a mudança é
// puramente aditiva — quem não conhece a characteristic nova simplesmente a
// ignora e continua funcionando.
constexpr char CHAR_PAN_UUID[] = "6e6e0008-3c17-4a2e-8f4b-1a2b3c4d5e6f";
// Dados de vibração do eixo de pan, em characteristic própria e com o mesmo
// formato de pacote da de tilt (índice + amostras int16 LE) — de novo para a
// mudança ser aditiva: um app que não a conhece continua recebendo a captura
// de tilt exatamente como antes.
constexpr char CHAR_VIBRATION_PAN_DATA_UUID[] = "6e6e0009-3c17-4a2e-8f4b-1a2b3c4d5e6f";
// Pedido de retransmissão de amostras da captura de vibração (v1.4.0).
// Write de 3 bytes: índice inicial (uint16 LE) + eixo (0 = tilt, 1 = pan).
// Existe porque notificação BLE não tem confirmação: se a fila do rádio
// encher, o pacote some em silêncio e o app fica com um buraco na série.
// Sem isto, o app não tinha como pedir o pedaço que faltou — e, pior,
// montava a série ignorando o buraco, o que deslocava todas as amostras
// seguintes no tempo e falseava o espectro inteiro.
constexpr char CHAR_VIBRATION_RESEND_UUID[] = "6e6e000a-3c17-4a2e-8f4b-1a2b3c4d5e6f";
// Write de 0x01 -> esquece os extremos dos dois eixos. Separada da de
// calibração porque são ações diferentes: calibrar move o zero, resetar os
// extremos não mexe na leitura.
constexpr char CHAR_RESET_PEAKS_UUID[] = "6e6e000b-3c17-4a2e-8f4b-1a2b3c4d5e6f";
// Read-only, diagnóstico do pan em campo (v1.6.3): 55 bytes LE = 9 float32
// (pan interno, offset, bias gy, bias gz, média gy, média gz, gy, gz, tilt)
// + uint32 amostras + uint32 falhas I2C + uint16 janelas fora do bias +
// uint8 bias pronto + uint32 leituras espúrias (v1.6.4) + uint32
// reconfigurações do MPU6050 (v1.6.5). Versões anteriores mandam menos bytes. Ver PanSensor::Diagnostics e
// python-app/tools/diagnostico_pan.py. Os apps não usam.
constexpr char CHAR_PAN_DIAGNOSTICS_UUID[] = "6e6e000d-3c17-4a2e-8f4b-1a2b3c4d5e6f";
// Extremos do peak-hold, num pacote só de 8 bytes (int16 LE, x ANGLE_SCALE):
// tiltMin, tiltMax, panMin, panMax. Os quatro juntos numa characteristic
// porque são sempre lidos juntos e mudam devagar — e, como sempre, aditivo:
// um app antigo simplesmente não a assina. UUID ...000c (não ...000a: já
// usado pela retransmissão de vibração acima).
constexpr char CHAR_PEAKS_UUID[] = "6e6e000c-3c17-4a2e-8f4b-1a2b3c4d5e6f";
constexpr uint32_t BLE_NOTIFY_INTERVAL_MS = 200;  // taxa de notificação do ângulo em modo contínuo
constexpr uint32_t BLE_VIBRATION_STATUS_NOTIFY_INTERVAL_MS = 300;  // limita notify() de status/progresso durante a captura
// Intervalo entre pacotes de dados da captura. Cada pacote leva 8 amostras,
// então 20ms davam só 400 amostras/s: uma captura de 500Hz por 12s (6000
// amostras em cada eixo, o teto do buffer) levaria 30s só para transferir,
// estourando o tempo limite do app. Em 5ms são 1600 amostras/s, e a perda
// eventual de pacote por fila cheia agora é recuperável (ver
// CHAR_VIBRATION_RESEND_UUID), em vez de corromper a série em silêncio.
constexpr uint32_t BLE_VIBRATION_CHUNK_INTERVAL_MS = 5;

// ============================================================================
// Compartilhados entre os dois transportes
// ============================================================================
constexpr float ANGLE_SCALE = 100.0f;  // valor no protocolo = ângulo * ANGLE_SCALE
constexpr float ANGLE_MIN_DEG = -60.0f;
constexpr float ANGLE_MAX_DEG = 60.0f;

// Correção do fator de escala da inclinação. Resolvido pelo atan2 entre os
// dois eixos do acelerômetro (sem singularidade nem necessidade de bias,
// diferente do giroscópio do pan), o erro residual observado em bancada
// ainda assim não era ruído: era uma escala sistemática, proporcional ao
// deslocamento em relação à calibração — mesma assinatura do erro do pan,
// mas aqui provavelmente por um descasamento de sensibilidade entre os
// dois eixos do acelerômetro usados no atan2 (Y e Z), e não por bias.
//
// Valor medido nesta unidade contra o inclinômetro de referência Mitutoyo
// PRO3600 (nº F4020023), 24 pontos entre -8,6° e +7,7° de inclinação
// (firmware 1.6.8): ajuste por escala pura (sem offset) = 0,951 x
// referência — desvio cai de 0,255° (sem correção) para 0,073° (com uma
// correção única para os dois lados).
//
// Firmware 1.6.10: o erro NÃO é simétrico entre inclinação positiva e
// negativa — separando os mesmos 24 pontos por sinal, o fator medido é
// 0,9593 do lado negativo e 0,9408 do lado positivo (uma correção única
// deixava um viés residual de -0,047°/-0,076° em cada lado). Assimetria
// assim não vem de um descasamento Y/Z puro (que seria simétrico, por
// atan2 ser função ímpar) — mais provável é um pequeno desalinhamento de
// montagem do sensor, ou o zero de calibração não coincidir exatamente com
// o zero mecânico. Corrigido com dois fatores, aplicados conforme o sinal
// do ÂNGULO BRUTO ABSOLUTO — o zero mecânico do sensor, e não o ponto de
// calibração (ver correctedAbsDeg() em AngleSensor.cpp: calibrar longe do
// zero mecânico não pode fazer a leitura escolher o lado errado).
//
// Firmware 1.6.11: o lado positivo foi recalibrado com 15 pontos novos
// (0,48° a 21,5°, complementando os 9 originais de até 6,99°) — o fator de
// 1,063 vinha ficando cada vez mais curto acima de uns 4°, chegando a
// -0,30° de erro em 21,5°. Com os 24 pontos positivos juntos (originais +
// novos, todos convertidos para bruto), o fator medido é 0,9291 em vez de
// 0,9408 — a extrapolação para além de 8,6° não era segura com tão poucos
// pontos. TILT_SCALE_CORRECTION_POS passa de 1,063 para 1,076: resíduo cai
// para -0,05°/+0,03° em toda a faixa até 21,5° (era até -0,30°).
//
// Firmware 1.6.12: mais uma rodada de bancada (0° a 8,03°, 15 pontos) ainda
// mostrava viés residual (-0,05° em média, até -0,14°) com o fator 1,076.
// Juntando as 3 rodadas (9 + 15 + 15 = 39 pontos positivos, 0° a 21,5°,
// tudo convertido para bruto): o fator medido é 0,9188. Testado um ajuste
// com deslocamento fixo além da escala — o termo de offset saiu em só
// 0,007°, confirmando que é escala pura, não um offset escondido.
// TILT_SCALE_CORRECTION_POS passa de 1,076 para 1,088: desvio-padrão do
// resíduo cai de 0,101° para 0,084° e o viés médio zera. O lado negativo
// continua com só os 9 pontos originais (até -8,6°) — mais pontos desse
// lado, numa faixa maior, teriam o mesmo benefício que o positivo já teve
// duas vezes. Refazer esta calibração (os dois lados, com mais pontos se
// possível) se o MPU6050 físico for substituído.
constexpr float TILT_SCALE_CORRECTION_NEG = 1.042f;  // 1 / 0,9593
constexpr float TILT_SCALE_CORRECTION_POS = 1.088f;  // 1 / 0,9188

// ============================================================================
// Filtro da leitura contínua (só do ângulo "normal" — o Modo Vibração NÃO
// passa por aqui, de propósito).
//
// Mesmo com o filtro interno do MPU6050 ligado (ver Mpu6050.h), uma amostra
// isolada do acelerômetro ainda carrega ruído suficiente para a leitura
// tremular nos centésimos de grau. A leitura de ângulo do dia a dia precisa
// ser estável; já a captura de vibração precisa justamente do sinal cru, com
// toda a sensibilidade — por isso os dois caminhos são separados em
// AngleSensor (readAngleDeg filtrado vs readRelativeAngleDeg instantâneo).
//
// O filtro é um "1-euro" (Casiez et al.): uma média móvel exponencial cuja
// frequência de corte NÃO é fixa — ela sobe quando o ângulo está mudando de
// verdade e desce quando o eixo está parado.
//
// Por que não uma média móvel comum: com constante de tempo fixa, estabilidade
// e rapidez são um cabo de guerra — mais suave significa mais lento, sempre.
// Com corte adaptativo dá para ter os dois, porque as duas situações são
// tratadas separadamente. Medido em simulação da cadeia completa (ruído do
// MPU6050 -> DLPF de 21Hz -> atan2 -> filtro), contra a média móvel de 0.5s
// que estava aqui antes:
//
//   ruído com o eixo parado : 0.016° -> 0.006° RMS  (2.5x menos)
//   atraso p/ acompanhar 90%
//   de um movimento real    : 0.63s  -> 0.21s       (3x mais rápido)
//
// Ou seja, melhor nos dois eixos ao mesmo tempo — não é uma troca.
//
// Ajuste dos três parâmetros:
// - MIN_CUTOFF manda no ruído com o eixo parado. Menor = mais estável, e é o
//   primeiro a mexer se a leitura ainda tremular no hardware real.
// - BETA manda na rapidez em movimento. Maior = acompanha mais rápido, mas
//   deixa passar mais ruído (o próprio ruído infla a derivada estimada).
// - DERIV_CUTOFF suaviza a estimativa de velocidade que alimenta o BETA.
//   Baixo de propósito: sem isso, o ruído infla a derivada, que levanta o
//   corte, que deixa passar mais ruído — a realimentação que estraga o ganho.
// ============================================================================
constexpr uint32_t ANGLE_SAMPLE_INTERVAL_MS = 10;  // 100Hz de amostragem interna
constexpr float ANGLE_FILTER_MIN_CUTOFF_HZ = 0.02f;
constexpr float ANGLE_FILTER_BETA = 0.15f;
constexpr float ANGLE_FILTER_DERIV_CUTOFF_HZ = 0.3f;

// ============================================================================
// Caminho de MEDIDA (mín/máx, histórico e relatório) — separado do caminho de
// TELA logo acima, e de propósito.
//
// O filtro da tela precisa ser pesado: o display tem passo de 0,25° e não pode
// tremular. Só que o mesmo valor suavizado alimentava os extremos — e aí uma
// rajada de vento real entrava no relatório achatada. Simulação da cadeia
// completa, rajada verdadeira de 2,0°, percentual do pico que chegava ao
// mín/máx:
//
//   duração da rajada       : 0,3s   0,5s   1,0s   2,0s
//   valor da tela, a 4-5 Hz :  24%    39%    64%    80%
//   este caminho, a 100 Hz  :  76%    88%    96%    99%
//
// São dois problemas somados: o filtro pesado achata o pico, e o app só
// amostra a 4-5 Hz, então nem o que sobra ele pega. Por isso o extremo é
// calculado aqui dentro, a 100 Hz (ver PeakHold.h), e transmitido pronto.
//
// PEAK_CUTOFF é o corte do filtro leve que alimenta o peak-hold. 3 Hz cobre a
// banda física do balanço do mastro (1-5 Hz) sem deixar passar o ruído de
// alta frequência do acelerômetro.
//
// PEAK_PERSIST_SAMPLES é o que impede o ruído restante de virar extremo: com
// o eixo parado por 10 min, a faixa mín/máx falsa sai de 0,34° (só o filtro
// de 3 Hz) para 0,24° (com 100 ms de persistência) — abaixo de um passo de
// tela — sem perder captura de rajada.
// ============================================================================
constexpr float ANGLE_PEAK_CUTOFF_HZ = 3.0f;
constexpr uint8_t ANGLE_PEAK_PERSIST_SAMPLES = 10;  // 10 amostras a 100 Hz = 100 ms

constexpr uint16_t VIBRATION_MAX_SAMPLES = 6000;  // limite de memória do buffer de captura

// Taxa máxima aceita numa captura de vibração. O limite real é o barramento
// I2C: cada amostra faz duas transações de 14 bytes (tilt e pan), que a
// 400kHz custam ~0,7ms somadas — em 500 amostras/s (2ms de período) a
// ocupação fica em torno de 35%, com folga para o resto do loop (Modbus,
// BLE). Acima disso o loop não fecha o período de forma confiável e as
// amostras sairiam com espaçamento irregular, o que distorceria o espectro.
constexpr uint16_t VIBRATION_MAX_RATE_HZ = 500;

// ============================================================================
// Eixo de azimute (pan) — medido pelo GIROSCÓPIO do MPU6050 (ver PanSensor.h).
//
// O acelerômetro é fisicamente cego a rotação em torno da vertical, mas o
// giroscópio não é: integrando a velocidade angular sai o ângulo de pan
// relativo ao zero calibrado. O que torna isso confiável aqui é o padrão de
// uso — o eixo fica parado a maior parte do tempo e se move em rajadas de
// poucos segundos — combinado com ZUPT (re-estimar o bias do giro sempre que
// está parado). Sem ZUPT, o bias viraria uma rampa de graus por minuto.
// ============================================================================

// Faixa mecânica do pan. PLACEHOLDER: o curso é limitado (sem volta
// completa), mas o valor exato depende da mecânica final — ajustar aqui
// quando estiver definido, como já foi feito quando a faixa do tilt mudou de
// 0-120° para -60~+60°. Limita também o integrador interno (anti-windup, ver
// PanSensor::clampIntegrator): passar do limite e voltar responde na hora.
constexpr float PAN_MIN_DEG = -90.0f;
constexpr float PAN_MAX_DEG = 90.0f;

// Janela de avaliação do ZUPT. Também define o menor movimento preservado:
// um deslocamento total abaixo de PAN_ZUPT_RATE_THRESHOLD_DPS * janela
// (1.0°/s * 1.0s = 1°) é interpretado como ruído e cancelado. Diminuir a
// janela melhora esse piso, mas piora a média usada para estimar o bias.
constexpr uint32_t PAN_ZUPT_WINDOW_MS = 1000;

// Limiar de "parado", aplicado à MÉDIA da janela (não ao pico). Usar a média
// deixa o detector imune a vibração, que é de média zero: o mastro pode estar
// balançando sob vento que a janela ainda é corretamente classificada como
// parada. A separação é confortável — o motor gira a ~20-30°/s.
constexpr float PAN_ZUPT_RATE_THRESHOLD_DPS = 1.0f;

// Segundo critério de "parado": nenhuma amostra da janela (já passada pela
// mediana) pode se afastar do bias mais que isto. Só com a média, um
// movimento que terminava logo no começo de uma janela (ex.: 30 ms a 20°/s
// = média de 0,6°/s) passava por parado e tinha até ~1° cancelado — perda
// que se acumulava a cada movimento. O motor gira a 20-30°/s, bem acima;
// vibração do mastro (~1°/s) fica bem abaixo, então a imunidade a
// vibração do critério da média continua valendo.
constexpr float PAN_ZUPT_PEAK_DPS = 8.0f;

// Velocidade com que o bias persegue a média das janelas paradas. Baixo de
// propósito: um movimento real lento o suficiente para passar pelo limiar
// acima precisaria persistir por muitas janelas para ser absorvido no bias.
constexpr float PAN_ZUPT_BIAS_ALPHA = 0.1f;

// O bias do giro deriva com a temperatura (o chip esquenta por minutos
// depois de ligado; o datasheet dá até ±20°/s na faixa de temperatura).
// Durante um giro não há janela parada para corrigi-lo, e com o bias parado
// no último valor a deriva virava erro de ângulo proporcional ao quadrado
// da duração do giro. Filtro alfa-beta: além do nível (ALPHA), estima a
// velocidade de deriva (BETA) e a extrapola entre janelas paradas — por no
// máximo PAN_BIAS_EXTRAPOLATION_MAX_S, e limitada a ±PAN_BIAS_SLOPE_MAX_DPS2.
constexpr float PAN_ZUPT_SLOPE_BETA = 0.01f;
constexpr float PAN_BIAS_SLOPE_MAX_DPS2 = 0.05f;
constexpr float PAN_BIAS_EXTRAPOLATION_MAX_S = 30.0f;

// Rede de segurança contra bias errado (ver "BOOT E RECUPERAÇÃO" em
// PanSensor.h): quantas janelas SEGUIDAS, coerentes entre si mas todas longe
// do bias, provam que o eixo está parado sob um bias errado. Tem de ser
// maior que a varredura mais longa possível a velocidade constante: 180° de
// curso a 20°/s (motor) duram 9 s — 20 janelas dão folga mais que dupla.
constexpr uint16_t PAN_BIAS_RELEARN_WINDOWS = 20;

// Diferença entre a amostra crua do giro e a mediana (ver "LEITURAS
// ESPÚRIAS" em PanSensor.h) acima da qual a amostra é contada como espúria
// no diagnóstico. Só conta: o descarte é feito pela mediana, sempre.
constexpr float PAN_SPIKE_REPORT_DPS = 30.0f;

// Correção do fator de escala do giroscópio (tolerância de fábrica ~+-3%).
// Este é o erro dominante depois que o ZUPT resolve o bias, e é proporcional
// ao deslocamento atual em relação ao zero — não se acumula com o tempo nem
// com o número de movimentos. Calibração de bancada: girar o eixo entre duas
// posições de separação angular conhecida e usar (ângulo real / integrado).
//
// Calibração PRELIMINAR (firmware 1.6.9), com só 2 pontos, contra mesa
// giratória/goniômetro com escala angular: alvo 8,00° -> lido 7,75°; alvo
// 9,00° -> lido 8,75° (erro crescendo com o deslocamento — assinatura de
// escala, não de offset — e batendo com a tolerância de fábrica do giro).
// Ajuste por escala pura: leitura = 0,9707 x referência (b = Σdr/Σr²).
// PAN_SCALE_CORRECTION = 1/0,9707 ≈ 1,030. Com a correção, o resíduo nos
// dois pontos cai para ±0,02° (de ±0,25° sem ela).
//
// Diferente do TILT_SCALE_CORRECTION (24 pontos contra um inclinômetro
// certificado ±0,02°), isto vem de só 2 pontos contra uma mesa/goniômetro de
// precisão não quantificada aqui — refinar com mais pontos (e idealmente
// mais afastados de zero, ex.: 45°/90°) quando possível, e sempre que o
// MPU6050 físico desta unidade for substituído.
constexpr float PAN_SCALE_CORRECTION = 1.030f;

// Teto para o dt de uma única integração. Protege contra um loop que atrasou
// muito (ou millis() dando a volta) virar um salto grande no ângulo. Não pode
// ser curto demais: leituras perdidas (falha de I2C, sensor sendo
// reconfigurado) viram um dt maior na amostra seguinte, integrada à taxa
// dela — com 0,1 s, um buraco de 0,3 s durante um giro a 20°/s perdia 4°.
constexpr float PAN_MAX_INTEGRATION_DT_S = 0.3f;

// Escala das amostras de vibração do eixo de pan no protocolo. Diferente do
// tilt, o que trafega aqui é VELOCIDADE ANGULAR (graus/s), não ângulo — ver
// "Modo Vibração no eixo de pan" em firmware/README.md. Com escala 100 e
// int16, a faixa vai a +-327°/s, cobrindo com folga o fundo de escala de
// +-250°/s do giroscópio: nenhum movimento fisicamente possível satura.
constexpr float PAN_RATE_SCALE = 100.0f;
