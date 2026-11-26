#include "AngleSensor.h"

#include <Arduino.h>
#include <math.h>

#include "Config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool AngleSensor::readRawAngleDeg(float &angleDeg) {
    float ax, ay, az;
    if (!_mpu.readAccelG(ax, ay, az)) {
        return false;
    }
    // Correção por eixo do acelerômetro (TILT_ACCEL_* em Config.h): ganho
    // relativo Y/Z e offsets, aplicados ANTES do atan2 — é aí que eles atuam
    // fisicamente. Com os valores padrão (1, 0, 0) é o atan2 puro de sempre.
    angleDeg = atan2((ay - TILT_ACCEL_OFFSET_Y_G) / TILT_ACCEL_GAIN_Y, az - TILT_ACCEL_OFFSET_Z_G) * 180.0f /
               static_cast<float>(M_PI);
    _lastRawDeg = angleDeg;
    return true;
}

bool AngleSensor::readFusionSample(float &angleDeg, float &rateDps, bool &gyroOk) {
    float ax, ay, az, gx, gy, gz;
    if (!_mpu.readMotion(ax, ay, az, gx, gy, gz)) {
        return false;
    }
    const float cy = (ay - TILT_ACCEL_OFFSET_Y_G) / TILT_ACCEL_GAIN_Y;
    const float cz = az - TILT_ACCEL_OFFSET_Z_G;
    angleDeg = atan2(cy, cz) * 180.0f / static_cast<float>(M_PI);
    _lastRawDeg = angleDeg;

    rateDps = 0.0f;
    gyroOk = false;
    // O bias do giro é copiado do PanSensor UMA vez, quando a fusão arma, e
    // não acompanhado depois: o PanSensor só cuida do bias no eixo do PAN — a
    // componente no eixo do tilt ele deixa andar (não usa), e com o tilt em
    // movimento constante ela chegou a 1,3 °/s de erro no simulador
    // (sim/regime_sim.cpp), o que virava 1,3° a mais de erro no tilt. Daqui
    // em diante quem acompanha a deriva é o termo integral da própria fusão.
    if (!_gyroBiasValid) {
        if (_pan == nullptr || !_pan->gyroBias(_gyroBias)) {
            return true;  // sem bias confirmado: só acelerômetro
        }
        _gyroBiasValid = true;
    }
    const Vec3 bias = _gyroBias;
    // Acelerômetro longe de 1 g = aceleração linear forte (tranco do motor do
    // PTU, pancada): a direção dele não é a vertical agora. Até a 1.6.33 a
    // fusão desistia do giro nessas amostras e saltava para o acelerômetro
    // sacudido; desde a 1.6.34 o giro segue sozinho, convertido com a última
    // vertical boa (a placa não gira dezenas de graus num tranco), e o
    // acelerômetro só volta a corrigir quando se acalma.
    const float norm = sqrtf(ax * ax + cy * cy + cz * cz);
    float ux, uy, uz;
    _accelOk = fabsf(norm - 1.0f) <= TILT_FUSION_ACCEL_TOL_G;
    if (_accelOk) {
        ux = ax / norm; uy = cy / norm; uz = cz / norm;
        _lastU[0] = ux; _lastU[1] = uy; _lastU[2] = uz;
        _hasLastU = true;
    } else if (_hasLastU) {
        ux = _lastU[0]; uy = _lastU[1]; uz = _lastU[2];
    } else {
        return true;
    }
    const float den = uy * uy + uz * uz;
    if (den < 0.05f) {
        return true;  // vertical quase no eixo X: o ângulo do tilt não é definido
    }
    const float wx = gx - bias.x, wy = gy - bias.y, wz = gz - bias.z;
    rateDps = wx - ux * (uz * wz + uy * wy) / den;
    gyroOk = true;
    return true;
}

float AngleSensor::emaAlpha(float cutoffHz, float dtS) {
    // tau = 1/(2*pi*fc); alpha = dt/(tau+dt)
    float tau = 1.0f / (2.0f * static_cast<float>(M_PI) * cutoffHz);
    return dtS / (tau + dtS);
}

void AngleSensor::update() {
    uint32_t now = millis();
    if (_filterReady && now - _lastSampleMs < ANGLE_SAMPLE_INTERVAL_MS) {
        return;
    }
    uint32_t elapsedMs = now - _lastSampleMs;
    _lastSampleMs = now;

    float rawDeg, gyroRateDps;
    bool gyroOk;
    if (!readFusionSample(rawDeg, gyroRateDps, gyroOk)) {
        return;  // falha de I2C: preserva o estado do filtro em vez de corrompê-lo
    }
    // Mediana da taxa do giro, como a do ângulo logo abaixo: uma leitura
    // espúria de centenas de °/s por 10 ms somaria graus ao ângulo fundido.
    for (int i = 0; i < FUSION_RATE_LEN - 1; i++) {
        _rateHist[i] = _rateHist[i + 1];
    }
    _rateHist[FUSION_RATE_LEN - 1] = gyroOk ? gyroRateDps : 0.0f;
    if (!gyroOk) {
        _rateHistCount = 0;  // sem giro agora: a mediana recomeça
    } else if (_rateHistCount < FUSION_RATE_LEN) {
        _rateHistCount++;
    }
    if (gyroOk && _rateHistCount == FUSION_RATE_LEN) {
        float sortedRate[FUSION_RATE_LEN];
        for (int i = 0; i < FUSION_RATE_LEN; i++) {
            float x = _rateHist[i];
            int j = i - 1;
            while (j >= 0 && sortedRate[j] > x) {
                sortedRate[j + 1] = sortedRate[j];
                j--;
            }
            sortedRate[j + 1] = x;
        }
        gyroRateDps = sortedRate[FUSION_RATE_LEN / 2];
    } else {
        gyroOk = false;
    }

    // Mediana das últimas 5 amostras ANTES do filtro: o 1-euro abre o corte
    // quando vê velocidade, então uma única amostra espúria (lixo do I2C)
    // era seguida quase inteira e depois demorava segundos para ser
    // esquecida — e ainda entrava nos extremos. Até juntar 5 amostras o
    // filtro nem começa, para uma amostra ruim no boot não virar o ponto
    // de partida.
    // Amostra com o acelerômetro sacudido (ver readFusionSample) não entra na
    // mediana: com o giro valendo, a fusão nem usa o acelerômetro nela.
    const bool accelOk = _accelOk || !gyroOk || !_filterReady;
    if (accelOk) {
        for (int i = 0; i < ANGLE_DESPIKE_LEN - 1; i++) {
            _rawHist[i] = _rawHist[i + 1];
        }
        _rawHist[ANGLE_DESPIKE_LEN - 1] = rawDeg;
    }
    if (_rawHistCount < ANGLE_DESPIKE_LEN) {
        _rawHistCount++;
        return;
    }
    float sorted[ANGLE_DESPIKE_LEN];
    for (int i = 0; i < ANGLE_DESPIKE_LEN; i++) {
        float x = _rawHist[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > x) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = x;
    }
    rawDeg = sorted[ANGLE_DESPIKE_LEN / 2];

    if (!_filterReady) {
        // Primeira amostra entra direto: sem isso a leitura começaria em 0°
        // e levaria segundos subindo até o valor real.
        fuse(rawDeg, 0.0f, false, 0.0f, true);
        _filteredRawDeg = rawDeg;
        _measuredRawDeg = rawDeg;
        _prevRawDeg = rawDeg;
        _filteredRateDps = 0.0f;
        _filterReady = true;
        return;
    }

    // dt vem do tempo REALMENTE decorrido, e não do intervalo nominal, para o
    // filtro manter o mesmo comportamento se o loop atrasar — ex: durante uma
    // captura de vibração, que divide o barramento I2C.
    float dtS = elapsedMs / 1000.0f;
    if (dtS <= 0.0f) {
        return;  // relógio não avançou: nada a integrar, e evita divisão por zero
    }

    // Fusão com o giroscópio (ver o cabeçalho de AngleSensor.h): daqui em
    // diante `rawDeg` é o ângulo fundido — os dois caminhos abaixo (tela e
    // medida) deixam de carregar a aceleração linear lida pelo acelerômetro.
    rawDeg = fuse(rawDeg, gyroRateDps, gyroOk, dtS, accelOk);

    // Filtro 1-euro (ver o bloco ANGLE_FILTER_* em Config.h). Primeiro estima
    // a velocidade angular e a suaviza; é ela que decide o quanto o filtro
    // deve "abrir".
    float rateDps = (rawDeg - _prevRawDeg) / dtS;
    _prevRawDeg = rawDeg;
    _filteredRateDps += emaAlpha(ANGLE_FILTER_DERIV_CUTOFF_HZ, dtS) * (rateDps - _filteredRateDps);

    // Parado, a velocidade é ~0 e o corte fica em MIN_CUTOFF (bem suave).
    // Em movimento real, o termo do BETA levanta o corte e o filtro acompanha.
    // Com a fusão ativa o sinal já chega com pouco ruído (o giro carrega o
    // movimento e o acelerômetro só corrige a 1/τ), e o corte mínimo pode ser
    // maior: com 0,02 Hz (τ = 8 s) qualquer resto ao parar — o tranco do motor
    // do PTU — levava ~10 s para sair da tela (bancada de 26/11/2026: até
    // 0,16° de diferença para o PRO-3600 lendo 3-5 s depois de parar).
    const float minCutoff = _fusionActive ? TILT_DISPLAY_MIN_CUTOFF_FUSED_HZ : ANGLE_FILTER_MIN_CUTOFF_HZ;
    float cutoffHz = minCutoff + ANGLE_FILTER_BETA * fabsf(_filteredRateDps);
    _filteredRawDeg += emaAlpha(cutoffHz, dtS) * (rawDeg - _filteredRawDeg);

    // Caminho de medida, em paralelo e a partir da MESMA amostra: filtro fixo
    // e leve, largo o bastante para não achatar rajada (ver o bloco de
    // ANGLE_PEAK_* em Config.h). O peak-hold é alimentado a 100 Hz — é essa
    // taxa, e não a de leitura do protocolo, que faz a diferença.
    _measuredRawDeg += emaAlpha(ANGLE_PEAK_CUTOFF_HZ, dtS) * (rawDeg - _measuredRawDeg);
    _peaks.push(toReported(_measuredRawDeg));
}

namespace {
// Fator de escala da inclinação, conforme o lado (ver TILT_SCALE_CORRECTION_*
// em Config.h) — a correção medida em bancada não é simétrica entre
// inclinação positiva e negativa. A assimetria é uma característica física
// do sensor/montagem (o atan2 muda de comportamento perto do zero MECÂNICO,
// não perto de onde o usuário decide calibrar), então o lado é escolhido
// pelo sinal do ângulo bruto (rawDeg, absoluto), e NÃO pelo sinal do
// deslocamento em relação à calibração — calibrar longe do zero mecânico
// não pode fazer a leitura escolher o lado errado.
float tiltScaleFor(float rawDeg) {
    return rawDeg < 0.0f ? TILT_SCALE_CORRECTION_NEG : TILT_SCALE_CORRECTION_POS;
}
// Ângulo absoluto já corrigido (antes de subtrair a calibração).
float correctedAbsDeg(float rawDeg) {
    return rawDeg * tiltScaleFor(rawDeg);
}
}  // namespace

float AngleSensor::fuse(float accelRawDeg, float rateDps, bool gyroOk, float dtS, bool accelOk) {
    const float accAbs = correctedAbsDeg(accelRawDeg);
    // Tranco (acelerômetro sacudido, giro bom): só o giro, sem correção nem
    // teste de divergência — o acelerômetro não é a vertical agora.
    if (gyroOk && !accelOk && _fusionActive && dtS > 0.0f && dtS <= TILT_FUSION_MAX_DT_S) {
        _fusedAbsDeg += (rateDps - _fusionBiasDps) * dtS;
        return _fusedAbsDeg / tiltScaleFor(_fusedAbsDeg);
    }
    // Sem giro utilizável, ou divergência grande demais entre os dois (bias
    // do giro errado, lixo no barramento): vale o acelerômetro, como antes.
    // A divergência tem de DURAR (TILT_FUSION_DIVERGE_SAMPLES): o resto de um
    // tranco que passou pelo teste de 1 g zerava o bias aprendido e jogava a
    // leitura no acelerômetro sacudido.
    if (fabsf(accAbs - _fusedAbsDeg) > TILT_FUSION_MAX_DIVERGENCE_DEG) {
        if (_divergeCount < 255) _divergeCount++;
    } else {
        _divergeCount = 0;
    }
    const bool diverged = _divergeCount >= TILT_FUSION_DIVERGE_SAMPLES;
    if (diverged && _fusionActive) {
        _gyroBiasValid = false;  // o bias copiado não serve mais: pega outro
    }
    if (!gyroOk || dtS <= 0.0f || dtS > TILT_FUSION_MAX_DT_S || diverged) {
        _fusedAbsDeg = accAbs;
        // O bias aprendido só é descartado na divergência (aí ele é o
        // suspeito). Numa falha passageira — leitura perdida no I2C, buraco
        // de amostragem — ele continua valendo: zerá-lo a cada falha deixava
        // a deriva térmica do giro inteira no ângulo (até 1,4° no soak_sim).
        if (diverged) {
            _fusionBiasDps = 0.0f;
        }
        _fusionActive = false;
        return accelRawDeg;
    }
    _fusionActive = true;
    const float predicted = _fusedAbsDeg + (rateDps - _fusionBiasDps) * dtS;
    const float err = accAbs - predicted;
    _fusedAbsDeg = predicted + (dtS / (TILT_FUSION_TAU_S + dtS)) * err;
    // Termo integral: aprende sozinho o bias do giro no eixo do tilt (deriva
    // térmica, resíduo do valor copiado), parado ou em movimento — o tilt tem
    // a gravidade como referência o tempo todo, não precisa de janela parada.
    _fusionBiasDps -= TILT_FUSION_KI * dtS * err;
    if (_fusionBiasDps > TILT_FUSION_MAX_BIAS_DPS) _fusionBiasDps = TILT_FUSION_MAX_BIAS_DPS;
    if (_fusionBiasDps < -TILT_FUSION_MAX_BIAS_DPS) _fusionBiasDps = -TILT_FUSION_MAX_BIAS_DPS;
    // De volta ao domínio "bruto" em que os filtros e toReported() trabalham
    // (inverso de correctedAbsDeg: os fatores são positivos, o sinal não muda).
    return _fusedAbsDeg / tiltScaleFor(_fusedAbsDeg);
}

float AngleSensor::readRelativeAngleDeg() {
    float rawDeg;
    if (!readRawAngleDeg(rawDeg)) {
        rawDeg = _lastRawDeg;  // amostra perdida: repete a última válida
    }
    return TILT_DIRECTION_SIGN * (correctedAbsDeg(rawDeg) - _offsetDeg);
}

float AngleSensor::toReported(float rawDeg) const {
    float angle = TILT_DIRECTION_SIGN * (correctedAbsDeg(rawDeg) - _offsetDeg);
    if (angle < ANGLE_MIN_DEG) angle = ANGLE_MIN_DEG;
    if (angle > ANGLE_MAX_DEG) angle = ANGLE_MAX_DEG;
    return angle;
}

float AngleSensor::readAngleDeg() {
    return toReported(_filteredRawDeg);
}

float AngleSensor::minAngleDeg() {
    return _peaks.hasData() ? _peaks.minValue() : readAngleDeg();
}

float AngleSensor::maxAngleDeg() {
    return _peaks.hasData() ? _peaks.maxValue() : readAngleDeg();
}

void AngleSensor::calibrate() {
    // _offsetDeg guarda o ângulo JÁ corrigido (não o bruto): assim toReported()
    // só precisa subtrair, e o lado (NEG/POS) de cada leitura continua sendo
    // decidido pelo sinal do ângulo bruto absoluto, nunca pelo da calibração.
    _offsetDeg = correctedAbsDeg(_filteredRawDeg);

    // Os extremos guardados são relativos ao zero antigo — mantê-los depois de
    // mover o zero reportaria mínimos e máximos que nunca aconteceram.
    _peaks.reset();
}
