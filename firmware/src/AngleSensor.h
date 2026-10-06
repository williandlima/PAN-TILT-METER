#pragma once

#include <stdint.h>

#include "Config.h"
#include "Mpu6050.h"
#include "PeakHold.h"

// Calcula o ângulo de inclinação (tilt) a partir do acelerômetro do
// MPU6050, usando atan2 entre dois eixos para manter sensibilidade
// praticamente uniforme em toda a faixa -60° a +60° — evita a região de baixa
// sensibilidade que a leitura de um único eixo teria perto de 90°
// (ver docs/fluxograma-python-app.md e a discussão do ensaio de vibração).
//
// Eixos e sinal usados aqui (Y, Z) são um ponto de partida — a
// confirmar/ajustar conforme a orientação real de montagem do sensor no
// pan-tilt, quando o projeto elétrico definir isso.
//
// Há TRÊS caminhos de leitura, de propósito — a mesma amostra de 100 Hz
// alimenta os três, cada um com o filtro que a sua finalidade pede:
// - readAngleDeg(): TELA. Filtro adaptativo "1-euro" (ver ANGLE_FILTER_* em
//   Config.h), pesado o bastante para o display de passo 0,25° não tremular;
// - minAngleDeg()/maxAngleDeg(): MEDIDA (mín/máx, histórico, relatório).
//   Filtro leve de ANGLE_PEAK_CUTOFF_HZ + peak-hold com teste de persistência
//   (ver PeakHold.h). Existe porque o valor da tela, suavizado e ainda lido
//   pelos apps a 4-5 Hz, registrava uma rajada real de 2° em 0,5 s como 0,78°;
// - readRelativeAngleDeg(): VIBRAÇÃO. Amostra instantânea, sem filtro nenhum —
//   ali a variação de frações de grau é justamente o que se quer medir, então
//   filtrar destruiria o dado.
//
// Os três caminhos aplicam TILT_SCALE_CORRECTION_NEG/_POS (Config.h) sobre o
// valor JÁ relativo à calibração — calibrado em bancada com um inclinômetro
// de referência (ver o comentário das constantes). Correção proporcional ao
// deslocamento em relação ao zero, mesmo raciocínio do pan, com um fator
// diferente para cada lado (a assimetria medida entre inclinação positiva e
// negativa não é ruído — ver Config.h).
class AngleSensor {
public:
    // O Mpu6050 é compartilhado (por referência) com o PanSensor: é o mesmo
    // chip físico, então quem o inicializa é o main(), uma vez só.
    explicit AngleSensor(Mpu6050 &mpu) : _mpu(mpu) {}

    // Chamar a cada iteração do loop(): amostra o sensor em intervalo fixo
    // (ANGLE_SAMPLE_INTERVAL_MS) e alimenta o filtro da leitura contínua.
    void update();

    // Ângulo filtrado, relativo à calibração (graus), já clampado a
    // [ANGLE_MIN_DEG, ANGLE_MAX_DEG] — usado na leitura "normal". Não faz
    // I2C: devolve o estado do filtro mantido por update().
    float readAngleDeg();

    // Amostra instantânea (uma leitura I2C na hora), relativa à calibração,
    // sem filtro nem clamp de faixa — usada na captura de vibração, onde a
    // variação em torno do zero pode ser negativa e a sensibilidade é o
    // objetivo.
    float readRelativeAngleDeg();

    // Extremos acumulados desde o último reset/calibração, já relativos à
    // calibração e clampados à faixa — o valor que os apps mostram como
    // mín/máx e que entra no relatório. Só fazem sentido se hasPeaks(); antes
    // disso (primeiros 100 ms após ligar ou calibrar) devolvem a leitura
    // corrente, para o app nunca exibir um extremo inventado.
    bool hasPeaks() const { return _peaks.hasData(); }
    float minAngleDeg();
    float maxAngleDeg();

    // Esquece os extremos sem mexer no zero — o botão de reset dos apps.
    void resetPeaks() { _peaks.reset(); }

    // Zera o offset de calibração na leitura atual (novo "zero" mecânico).
    // Usa o valor filtrado, não uma amostra isolada, para a calibração ser
    // repetível mesmo com o sensor sob vibração. Zera junto os extremos: com
    // o zero em outro lugar, os extremos antigos não significariam mais nada.
    void calibrate();

private:
    Mpu6050 &_mpu;
    // Ângulo JÁ CORRIGIDO (não bruto) no instante da calibração — ver
    // AngleSensor::calibrate()/correctedAbsDeg() no .cpp. Guardar o valor
    // corrigido, e não o bruto, é o que permite escolher o lado da correção
    // (NEG/POS) pelo sinal do ângulo bruto absoluto em vez do sinal relativo
    // à calibração — calibrar fora do zero mecânico não muda qual lado é
    // usado numa leitura.
    float _offsetDeg = 0.0f;

    // Estado do filtro 1-euro (todos em ângulo absoluto, antes do offset):
    float _filteredRawDeg = 0.0f;   // saída do filtro
    float _filteredRateDps = 0.0f;  // derivada suavizada, que comanda o corte
    float _prevRawDeg = 0.0f;       // amostra anterior, para calcular a derivada
    float _lastRawDeg = 0.0f;       // última amostra válida (absoluta)
    bool _filterReady = false;      // primeira amostra inicializa o filtro direto

    // Mediana móvel do ângulo cru, antes do filtro (ver update()).
    static constexpr int ANGLE_DESPIKE_LEN = 5;
    float _rawHist[ANGLE_DESPIKE_LEN] = {};
    int _rawHistCount = 0;
    uint32_t _lastSampleMs = 0;

    // Caminho de medida: filtro leve de 1 polo em ANGLE_PEAK_CUTOFF_HZ,
    // independente do 1-euro acima, alimentando o peak-hold.
    float _measuredRawDeg = 0.0f;
    PeakHold _peaks{ANGLE_PEAK_PERSIST_SAMPLES};

    // Lê uma amostra e converte para ângulo absoluto. Retorna false se a
    // leitura I2C falhar (sensor desconectado, mau contato).
    bool readRawAngleDeg(float &angleDeg);

    // Aplica o offset de calibração, o fator de escala — diferente para cada
    // lado (TILT_SCALE_CORRECTION_NEG/_POS) — e o clamp de faixa: a forma em
    // que o ângulo sai deste sensor, seja como leitura corrente ou extremo.
    float toReported(float rawDeg) const;

    // Coeficiente de uma média móvel exponencial de 1 polo para a frequência
    // de corte e o intervalo dados.
    static float emaAlpha(float cutoffHz, float dtS);
};
