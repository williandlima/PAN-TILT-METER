#include "PanSensor.h"

#include <Arduino.h>
#include <math.h>

#include "Config.h"

bool PanSensor::sampleMotion(float &gyDps, float &gzDps, float &tiltRad) {
    float ax, ay, az, gxDps;
    if (!_mpu.readMotion(ax, ay, az, gxDps, gyDps, gzDps)) {
        return false;
    }
    // Tilt do MESMO burst: sem defasagem entre o ângulo usado na projeção e
    // as taxas projetadas.
    tiltRad = atan2f(ay, az);
    return true;
}

float PanSensor::panRateDps(float gyDps, float gzDps, float tiltRad) const {
    // Projeção da velocidade angular sobre a vertical, escrita em coordenadas
    // do corpo (item 1 do cabeçalho de PanSensor.h), com o bias de cada eixo
    // removido ANTES de projetar (item 2).
    return projectedDps(gyDps - _biasGyDps, gzDps - _biasGzDps, tiltRad);
}

float PanSensor::readInstantRateDps() {
    float gyDps, gzDps, tiltRad;
    if (!sampleMotion(gyDps, gzDps, tiltRad)) {
        return _lastRateDps;  // amostra perdida: repete a última válida
    }
    _lastRateDps = panRateDps(gyDps, gzDps, tiltRad) * PAN_SCALE_CORRECTION;
    return _lastRateDps;
}

namespace {
// Mediana de n <= 5 valores (média dos dois centrais se n for par).
float medianOf(const float *v, int n) {
    float s[5];
    for (int i = 0; i < n; i++) {
        float x = v[i];
        int j = i - 1;
        while (j >= 0 && s[j] > x) {
            s[j + 1] = s[j];
            j--;
        }
        s[j + 1] = x;
    }
    return (n % 2) ? s[n / 2] : 0.5f * (s[n / 2 - 1] + s[n / 2]);
}
}  // namespace

bool PanSensor::despike(float &gyDps, float &gzDps, float &tiltRad) {
    for (int i = 0; i < PAN_DESPIKE_LEN - 1; i++) {
        _gyHist[i] = _gyHist[i + 1];
        _gzHist[i] = _gzHist[i + 1];
        _tiltHist[i] = _tiltHist[i + 1];
    }
    _gyHist[PAN_DESPIKE_LEN - 1] = gyDps;
    _gzHist[PAN_DESPIKE_LEN - 1] = gzDps;
    _tiltHist[PAN_DESPIKE_LEN - 1] = tiltRad;
    if (_histCount < PAN_DESPIKE_LEN) {
        _histCount++;
    }
    // Só com o histórico cheio a mediana rejeita até 2 amostras ruins
    // seguidas (picos vêm em dupla); antes disso, melhor esperar.
    if (_histCount < PAN_DESPIKE_LEN) {
        return false;
    }
    int first = PAN_DESPIKE_LEN - _histCount;
    float gy = medianOf(_gyHist + first, _histCount);
    float gz = medianOf(_gzHist + first, _histCount);
    if (fabsf(gy - gyDps) > PAN_SPIKE_REPORT_DPS || fabsf(gz - gzDps) > PAN_SPIKE_REPORT_DPS) {
        _spikeCount++;
    }
    gyDps = gy;
    gzDps = gz;
    // O tilt da projeção vem do MESMO quadro; um quadro de lixo que passe
    // pelo teste de plausibilidade traz também um tilt errado.
    tiltRad = medianOf(_tiltHist + first, _histCount);
    return true;
}

void PanSensor::update() {
    uint32_t now = millis();
    if (_hasLastSample && now - _lastSampleMs < ANGLE_SAMPLE_INTERVAL_MS) {
        return;
    }
    if (_failedAttempt && now == _lastAttemptMs) {
        return;  // falhou neste mesmo ms: tenta de novo no próximo
    }

    float gyDps, gzDps, tiltRad;
    if (!sampleMotion(gyDps, gzDps, tiltRad)) {
        // O relógio da integração NÃO avança numa falha: o intervalo fica
        // para a próxima leitura boa, que o integra inteiro. Avançando aqui
        // (como até a 1.6.4), cada leitura perdida durante um giro jogava
        // fora 10 ms de rotação (0,2° a 20°/s) — em silêncio, a cada falha.
        _i2cFailures++;
        _failedAttempt = true;
        _lastAttemptMs = now;
        return;  // falha de I2C: preserva o estado em vez de corrompê-lo
    }
    // Salto impossível desde a última amostra boa (ver PAN_GYRO_SLEW_MAX_DPS):
    // quadro de lixo do barramento que passou pelo teste do acelerômetro.
    // Descartado como uma falha de leitura — nem entra na mediana, que só
    // dá conta de 2 seguidos. Depois de PAN_GYRO_SLEW_MAX_REJECTS seguidos, a
    // referência é que está velha: aceita.
    if (_hasLastSample && now - _lastSampleMs <= 3 * ANGLE_SAMPLE_INTERVAL_MS &&
        _slewRejects < PAN_GYRO_SLEW_MAX_REJECTS &&
        (fabsf(gyDps - _lastGyDps) > PAN_GYRO_SLEW_MAX_DPS || fabsf(gzDps - _lastGzDps) > PAN_GYRO_SLEW_MAX_DPS)) {
        _slewRejects++;
        _spikeCount++;
        _failedAttempt = true;
        _lastAttemptMs = now;
        return;
    }
    _slewRejects = 0;
    _failedAttempt = false;
    uint32_t elapsedMs = now - _lastSampleMs;
    _lastSampleMs = now;
    _lastGyDps = gyDps;
    _lastGzDps = gzDps;
    _lastTiltRad = tiltRad;
    _sampleCount++;
    if (!_hasLastSample) {
        // Primeira amostra: sem intervalo anterior, não há dt para integrar.
        _hasLastSample = true;
        resetWindow(now);
        despike(gyDps, gzDps, tiltRad);
        return;
    }

    float dtS = elapsedMs / 1000.0f;
    if (dtS > PAN_MAX_INTEGRATION_DT_S) {
        dtS = PAN_MAX_INTEGRATION_DT_S;
    }

    // Depois de um buraco (loop atrasado, leituras perdidas), o histórico da
    // mediana é de ANTES dele: no fim de um giro, 4 amostras velhas a 20°/s
    // venciam a nova já parada e o buraco inteiro era integrado a 20°/s. Ele
    // é descartado, e o buraco só é integrado quando 5 amostras novas dão
    // uma mediana confiável — nunca com uma amostra isolada, que pode ser
    // justamente a espúria.
    if (elapsedMs > 3 * ANGLE_SAMPLE_INTERVAL_MS) {
        _histCount = 0;
        _hasPrevRaw = false;
    }
    float rawGyDps = gyDps;
    float rawGzDps = gzDps;
    if (!despike(gyDps, gzDps, tiltRad)) {
        _pendingDtS += dtS;
        return;
    }
    dtS += _pendingDtS;
    _pendingDtS = 0.0f;

    // Só integra depois que a primeira janela estabeleceu o bias: antes
    // disso, o zero-rate de fábrica (±20°/s) jogaria o ângulo longe.
    if (_biasReady && _sinceStillS < PAN_BIAS_EXTRAPOLATION_MAX_S) {
        // Deriva térmica do bias continua durante o movimento, quando não
        // há janela parada para corrigi-lo: segue a velocidade estimada.
        // Só por um tempo limitado — a estimativa envelhece.
        _biasGyDps += _biasSlopeGy * dtS;
        _biasGzDps += _biasSlopeGz * dtS;
        _sinceStillS += dtS;
    }

    if (_biasReady) {
        // Regra do trapézio: a taxa muda entre duas amostras (início e fim de
        // cada giro), e multiplicar só a taxa nova pelo intervalo inteiro
        // errava até uma amostra (~0,2° a 20°/s) por borda de movimento.
        float rate = panRateDps(gyDps, gzDps, tiltRad) * PAN_SCALE_CORRECTION;
        float prevRate = _hasPrevIntegRate ? _prevIntegRateDps : rate;
        _prevIntegRateDps = rate;
        _hasPrevIntegRate = true;
        float before = _panDeg;
        _panDeg += 0.5f * (prevRate + rate) * dtS;
        clampIntegrator();
        // O que foi EFETIVAMENTE aplicado (após o clamp), para o cancelamento
        // da janela desfazer exatamente isso.
        _windowDeltaDeg += _panDeg - before;
    }

    _windowGySumDps += gyDps;
    _windowGzSumDps += gzDps;
    _windowSamples++;
    if (_biasReady) {
        // Medida de vibração: diferença entre amostras BRUTAS sucessivas
        // (antes da mediana). Bruta porque a mediana retifica vibração perto
        // de submúltiplos da taxa de amostragem (ex.: 33 Hz = 100/3) num
        // nível quase constante — some a oscilação, fica um desvio falso na
        // média. Diferença sucessiva porque mede só o que muda de uma amostra
        // para a outra: o próprio giro, mesmo acelerando, quase não aparece
        // nela. Os picos que a mediana rejeita ficam de fora. Ver
        // PAN_ZUPT_VIBRATION_FACTOR.
        bool rawOk = fabsf(rawGyDps - gyDps) <= PAN_SPIKE_REPORT_DPS && fabsf(rawGzDps - gzDps) <= PAN_SPIKE_REPORT_DPS;
        if (rawOk && _hasPrevRaw) {
            float dGy = rawGyDps - _prevRawGyDps;
            float dGz = rawGzDps - _prevRawGzDps;
            _windowDiffSqSum += dGy * dGy + dGz * dGz;
            _windowDiffCount++;
        }
        _hasPrevRaw = rawOk;
        _prevRawGyDps = rawGyDps;
        _prevRawGzDps = rawGzDps;
        float devGy = gyDps - _biasGyDps;
        float devGz = gzDps - _biasGzDps;
        float peak = hypotf(devGy, devGz);
        if (peak > _windowPeakDps) {
            _windowPeakDps = peak;
        }
    }

    if (now - _windowStartMs >= PAN_ZUPT_WINDOW_MS) {
        closeWindow(now);
    }

    // Extremos amostrados a 100 Hz, e não na cadência do protocolo: uma
    // varredura do motor a 20-30°/s atravessa vários graus entre duas leituras
    // do app. Ver minPanDeg() em PanSensor.h.
    _peaks.push(readPanDeg());
}

void PanSensor::closeWindow(uint32_t now) {
    if (_windowSamples == 0) {
        resetWindow(now);
        return;
    }

    float meanGyDps = _windowGySumDps / _windowSamples;
    float meanGzDps = _windowGzSumDps / _windowSamples;
    float prevGyDps = _prevMeanGyDps;
    float prevGzDps = _prevMeanGzDps;
    bool hadPrev = _hasPrevMean;
    _prevMeanGyDps = meanGyDps;
    _prevMeanGzDps = meanGzDps;
    _hasPrevMean = true;

    if (!_biasReady) {
        // Adoção: duas janelas seguidas com a mesma média. Ver "BOOT E
        // RECUPERAÇÃO" em PanSensor.h.
        if (hadPrev) {
            float spread = hypotf(meanGyDps - prevGyDps, meanGzDps - prevGzDps);
            if (spread < PAN_BIAS_ADOPT_COHERENCE_DPS) {
                adoptBias(0.5f * (meanGyDps + prevGyDps), 0.5f * (meanGzDps + prevGzDps), 0.0f, 0.0f,
                          spread < PAN_ZUPT_RATE_THRESHOLD_DPS);
            }
        }
        resetWindow(now);
        return;
    }

    // A média da janela corresponde ao bias do MEIO da janela; o bias
    // corrente já foi extrapolado até o fim dela.
    float winS = (now - _windowStartMs) / 1000.0f;
    float dGy = meanGyDps - (_biasGyDps - _biasSlopeGy * 0.5f * winS);
    float dGz = meanGzDps - (_biasGzDps - _biasSlopeGz * 0.5f * winS);
    // Dispersão das amostras dentro da janela: sob vibração, a média de uma
    // janela de 1 s não zera (fração de período que sobra), e o resíduo é
    // proporcional à amplitude — o limiar acompanha (PAN_ZUPT_VIBRATION_FACTOR).
    // Para ruído branco ou vibração, sqrt(média do quadrado da diferença /
    // 2) é a dispersão da parte rápida do sinal; descontado o ruído próprio
    // do sensor (em quadratura), sobra a vibração.
    float fastVar = _windowDiffCount > 0 ? _windowDiffSqSum / (2.0f * _windowDiffCount) : 0.0f;
    float vibVar = fastVar - PAN_ZUPT_NOISE_SPREAD_DPS * PAN_ZUPT_NOISE_SPREAD_DPS;
    float spreadDps = vibVar > 0.0f ? sqrtf(vibVar) : 0.0f;
    float threshold = fmaxf(PAN_ZUPT_RATE_THRESHOLD_DPS, PAN_ZUPT_VIBRATION_FACTOR * spreadDps);
    threshold = fmaxf(threshold, PAN_ZUPT_WANDER_FACTOR * sqrtf(_wanderVar));
    bool prevStill = _prevStill;
    _prevStill = false;
    // Desvio com sinal, projetado na vertical: o sentido do giro.
    float signedDps = projectedDps(dGy, dGz, _lastTiltRad);
    bool stillCandidate = hypotf(dGy, dGz) < threshold && _windowPeakDps < PAN_ZUPT_PEAK_DPS;

    if (stillCandidate && _prevMoving && _tailWindows < PAN_ZUPT_TAIL_MAX_WINDOWS &&
        isEdgeOf(signedDps, spreadDps, _lastMovingDps)) {
        // Cauda de um giro que está freando devagar (à mão): ainda é o mesmo
        // movimento — mantém o que foi integrado e não ensina o bias.
        _tailWindows++;
        _lastMovingDps = signedDps;
        pushRelearnWindow(meanGyDps, meanGzDps, _windowDeltaDeg, spreadDps);
        pushExcursion({meanGyDps, meanGzDps, _windowDeltaDeg, winS, _lastTiltRad, spreadDps, hypotf(dGy, dGz),
                       _biasGyDps - _biasSlopeGy * 0.5f * winS, _biasGzDps - _biasSlopeGz * 0.5f * winS, now, true});
        tryShiftEscape();
        resetWindow(now);
        return;
    }

    if (stillCandidate) {
        // Janela parada: refina o bias (filtro alfa-beta: nível e velocidade
        // de deriva) e desfaz o que foi integrado nela, para o ruído do giro
        // não virar random walk enquanto parado.
        pushBiasUndo(now, winS, signedDps, spreadDps, meanGyDps, meanGzDps);
        if (prevStill && hadPrev) {
            learnWander(meanGyDps - prevGyDps, meanGzDps - prevGzDps);
        }
        _prevStill = true;
        _excCount = 0;
        _prevMoving = false;
        _tailWindows = 0;
        _biasGyDps += PAN_ZUPT_BIAS_ALPHA * dGy;
        _biasGzDps += PAN_ZUPT_BIAS_ALPHA * dGz;
        if (winS > 0.0f) {
            _biasSlopeGy = clampSlope(_biasSlopeGy + PAN_ZUPT_SLOPE_BETA * dGy / winS);
            _biasSlopeGz = clampSlope(_biasSlopeGz + PAN_ZUPT_SLOPE_BETA * dGz / winS);
        }
        _sinceStillS = 0.0f;
        _panDeg -= _windowDeltaDeg;
        clampIntegrator();
        _biasConfirmed = true;
        _relearnCount = 0;
    } else {
        // Movimento: o que foi integrado fica, e o bias NÃO aprende com esta
        // janela — aprender aqui era o que contaminava o bias com a própria
        // velocidade de um giro lento (ver PAN_ZUPT_RATE_THRESHOLD_DPS). As
        // últimas janelas "paradas" podem ter sido o começo suave deste mesmo
        // giro: o que elas ensinaram ao bias é desfeito.
        undoRecentBiasLearning(now, winS, signedDps);
        _prevMoving = true;
        _tailWindows = 0;
        _lastMovingDps = signedDps;
        bool quiet = hypotf(dGy, dGz) < fmaxf(PAN_ZUPT_SHIFT_MAX_DPS, PAN_ZUPT_VIBRATION_FACTOR * spreadDps) &&
                     _windowPeakDps < PAN_ZUPT_PEAK_DPS;
        pushRelearnWindow(meanGyDps, meanGzDps, _windowDeltaDeg, spreadDps);
        pushExcursion({meanGyDps, meanGzDps, _windowDeltaDeg, winS, _lastTiltRad, spreadDps, hypotf(dGy, dGz),
                       _biasGyDps - _biasSlopeGy * 0.5f * winS, _biasGzDps - _biasSlopeGz * 0.5f * winS, now, quiet});
        tryRelearn(winS);
        if (_excCount > 0) {  // tryRelearn() não adotou
            tryShiftEscape();
        }
    }

    resetWindow(now);
}

bool PanSensor::isEdgeOf(float edgeDps, float spreadDps, float motionDps) {
    // Borda (começo ou fim suave) de um giro: mesmo sentido, acima do ruído,
    // acima do resíduo de vibração e mais lenta que o giro em si. Deriva
    // térmica não ultrapassa o degrau: é praticamente igual janela a janela.
    if ((edgeDps > 0.0f) != (motionDps > 0.0f)) return false;
    float edge = fabsf(edgeDps);
    if (edge < PAN_ZUPT_EDGE_MIN_DPS) return false;
    if (edge < PAN_ZUPT_VIBRATION_FACTOR * spreadDps) return false;
    return edge <= fabsf(motionDps) - PAN_ZUPT_EDGE_STEP_DPS;
}

void PanSensor::pushBiasUndo(uint32_t now, float winS, float signedDps, float spreadDps, float meanGy, float meanGz) {
    if (_undoCount == UNDO_LEN) {
        for (int i = 0; i < UNDO_LEN - 1; i++) {
            _undo[i] = _undo[i + 1];
        }
        _undoCount--;
    }
    _undo[_undoCount++] = BiasSnapshot{_biasGyDps, _biasGzDps, _biasSlopeGy, _biasSlopeGz, now, _sinceStillS,
                                       _windowDeltaDeg, signedDps, spreadDps, winS, _lastTiltRad, meanGy, meanGz};
}

float PanSensor::projectedDps(float dGy, float dGz, float tiltRad) {
    // Projeção na vertical (positivo = anti-horário visto de cima), no sentido
    // do pan do usuário (PAN_DIRECTION_SIGN). Toda conta que soma ao ângulo
    // passa por aqui, para o sentido ser um só.
    return PAN_DIRECTION_SIGN * (dGz * cosf(tiltRad) + dGy * sinf(tiltRad));
}

void PanSensor::undoRecentBiasLearning(uint32_t now, float winS, float motionDps) {
    int count = _undoCount;
    _undoCount = 0;  // a sequência parada acabou, use-se ou não o que ela guardou
    if (count == 0) {
        return;
    }
    // Enquanto o começo suave de um giro ainda passava por "parado", o bias o
    // perseguia: medido contra o bias de cada época, o desvio de cada janela
    // sai achatado. Por isso todos são reescritos contra UMA referência — o
    // bias usado na janela mais antiga guardada, extrapolado pela deriva dele.
    // Trocar o bias b por r numa janela muda o desvio em proj(b - r).
    const BiasSnapshot &ref = _undo[0];
    auto refGyAt = [&](uint32_t ms, float halfS) { return ref.gy + ref.slopeGy * ((ms - ref.ms) / 1000.0f - halfS); };
    auto refGzAt = [&](uint32_t ms, float halfS) { return ref.gz + ref.slopeGz * ((ms - ref.ms) / 1000.0f - halfS); };
    float devRef[UNDO_LEN];
    for (int k = 0; k < count; k++) {
        const BiasSnapshot &w = _undo[k];
        float h = 0.5f * w.winS;
        devRef[k] = w.signedDps + projectedDps((w.gy - w.slopeGy * h) - refGyAt(w.ms, h),
                                               (w.gz - w.slopeGz * h) - refGzAt(w.ms, h), w.tiltRad);
    }
    float h = 0.5f * winS;
    float limit = motionDps + projectedDps((_biasGyDps - _biasSlopeGy * h) - refGyAt(now, h),
                                           (_biasGzDps - _biasSlopeGz * h) - refGzAt(now, h), _lastTiltRad);

    // As janelas logo antes que já mostravam este giro começando (mesmo
    // sentido, cada vez mais rápido) eram ele. Sem esse sinal, as janelas
    // estavam paradas de verdade e o que ensinaram ao bias (deriva térmica)
    // fica.
    int first = count;
    for (int k = count - 1; k >= 0; k--) {
        if (!isEdgeOf(devRef[k], _undo[k].spreadDps, limit)) {
            break;
        }
        first = k;
        limit = devRef[k];
    }
    if (first == count) {
        return;
    }

    // Volta ao bias de um pouco antes da primeira dessas janelas (as que
    // vieram logo antes dela já eram o giro, lento demais para o critério de
    // borda, e também ensinaram o bias), devolve o que foi cancelado nelas e
    // corrige o que foi integrado com o bias contaminado — nelas e nesta.
    int revertTo = first - PAN_ZUPT_EDGE_PRE_WINDOWS;
    if (revertTo < 0) revertTo = 0;
    const BiasSnapshot &s = _undo[revertTo];
    auto revGyAt = [&](uint32_t ms, float halfS) { return s.gy + s.slopeGy * ((ms - s.ms) / 1000.0f - halfS); };
    auto revGzAt = [&](uint32_t ms, float halfS) { return s.gz + s.slopeGz * ((ms - s.ms) / 1000.0f - halfS); };
    float before = _panDeg;
    for (int k = revertTo; k < count; k++) {
        const BiasSnapshot &w = _undo[k];
        float hk = 0.5f * w.winS;
        // Só a cadeia volta; antes dela, só a correção de bias (a janela foi
        // cancelada, então não há o que corrigir nela).
        if (k < first) continue;
        float restored = w.deltaDeg + projectedDps((w.gy - w.slopeGy * hk) - revGyAt(w.ms, hk),
                                                   (w.gz - w.slopeGz * hk) - revGzAt(w.ms, hk), w.tiltRad) *
                                          w.winS * PAN_SCALE_CORRECTION;
        _panDeg += restored;
        // Voltaram a ser giro: fazem parte da excursão que começa aqui.
        pushExcursion({w.meanGy, w.meanGz, restored, w.winS, w.tiltRad, w.spreadDps, fabsf(w.signedDps),
                       revGyAt(w.ms, hk), revGzAt(w.ms, hk), w.ms, true});
    }
    _panDeg += projectedDps((_biasGyDps - _biasSlopeGy * h) - revGyAt(now, h),
                            (_biasGzDps - _biasSlopeGz * h) - revGzAt(now, h), _lastTiltRad) *
               winS * PAN_SCALE_CORRECTION;
    clampIntegrator();
    _windowDeltaDeg += _panDeg - before;

    // O snapshot guarda o bias no FIM da janela `first` (antes da atualização
    // dela); daqui segue extrapolado.
    _biasGyDps = revGyAt(now, 0.0f);
    _biasGzDps = revGzAt(now, 0.0f);
    _biasSlopeGy = s.slopeGy;
    _biasSlopeGz = s.slopeGz;
    _sinceStillS = s.sinceStillS + (now - s.ms) / 1000.0f;
}

void PanSensor::pushRelearnWindow(float meanGyDps, float meanGzDps, float deltaDeg, float spreadDps) {
    _relearnGy[_relearnHead] = meanGyDps;
    _relearnGz[_relearnHead] = meanGzDps;
    _relearnDeltaDeg[_relearnHead] = deltaDeg;
    _relearnSpreadDps[_relearnHead] = spreadDps;
    _relearnHead = (_relearnHead + 1) % RELEARN_LEN;
    if (_relearnCount < RELEARN_LEN) {
        _relearnCount++;
    }
}

void PanSensor::tryRelearn(float winS) {
    if (_relearnCount < RELEARN_LEN) {
        return;
    }
    // Reta de mínimos quadrados das médias (índice 0 = janela mais antiga):
    // bias constante ou derivando com a temperatura cai sobre ela; um giro de
    // verdade, com aceleração e frenagem, não.
    const float n = static_cast<float>(RELEARN_LEN);
    const float xMean = 0.5f * (n - 1.0f);
    float gyMean = 0.0f, gzMean = 0.0f, sxx = 0.0f, sxGy = 0.0f, sxGz = 0.0f, erase = 0.0f;
    for (int k = 0; k < RELEARN_LEN; k++) {
        int i = (_relearnHead + k) % RELEARN_LEN;
        gyMean += _relearnGy[i];
        gzMean += _relearnGz[i];
        erase += _relearnDeltaDeg[i];
    }
    gyMean /= n;
    gzMean /= n;
    for (int k = 0; k < RELEARN_LEN; k++) {
        int i = (_relearnHead + k) % RELEARN_LEN;
        float dx = k - xMean;
        sxx += dx * dx;
        sxGy += dx * (_relearnGy[i] - gyMean);
        sxGz += dx * (_relearnGz[i] - gzMean);
    }
    float bGy = sxGy / sxx;  // °/s por janela
    float bGz = sxGz / sxx;
    float ss = 0.0f;
    for (int k = 0; k < RELEARN_LEN; k++) {
        int i = (_relearnHead + k) % RELEARN_LEN;
        float dx = k - xMean;
        float rGy = _relearnGy[i] - (gyMean + bGy * dx);
        float rGz = _relearnGz[i] - (gzMean + bGz * dx);
        ss += rGy * rGy + rGz * rGz;
    }
    // Sob vibração as médias ondulam em torno da reta (ver
    // PAN_ZUPT_VIBRATION_FACTOR), e um limite fixo nunca seria atingido: um
    // bias errado ficaria para sempre. O limite cresce com a vibração.
    float spreadMean = 0.0f;
    for (int k = 0; k < RELEARN_LEN; k++) spreadMean += _relearnSpreadDps[k];
    spreadMean /= n;
    float limit = _biasConfirmed ? PAN_BIAS_RELEARN_RESIDUAL_DPS : PAN_BIAS_RELEARN_RESIDUAL_PROVISIONAL_DPS;
    limit = fmaxf(limit, PAN_BIAS_RELEARN_VIBRATION_FACTOR * spreadMean);
    if (sqrtf(ss / n) >= limit) {
        return;
    }
    // Parado sob bias errado: desfaz o que essas janelas integraram e adota
    // o bias da reta no fim da última janela (a média dela é o meio).
    _panDeg -= erase;
    clampIntegrator();
    float tEnd = (n - 1.0f) - xMean + 0.5f;
    float perS = winS > 0.0f ? 1.0f / winS : 0.0f;
    adoptBias(gyMean + bGy * tEnd, gzMean + bGz * tEnd, clampSlope(bGy * perS), clampSlope(bGz * perS), false);
}

void PanSensor::learnWander(float stepGy, float stepGz) {
    _wanderVar += (stepGy * stepGy + stepGz * stepGz - _wanderVar) / PAN_ZUPT_WANDER_WINDOWS;
}

void PanSensor::pushExcursion(const ExcursionWindow &w) {
    if (_excCount == EXC_LEN) {
        for (int i = 0; i < EXC_LEN - 1; i++) {
            _exc[i] = _exc[i + 1];
        }
        _excCount--;
    }
    _exc[_excCount++] = w;
}

bool PanSensor::tryShiftEscape() {
    const int m = PAN_ZUPT_SHIFT_WINDOWS;
    if (_excCount < m) {
        return false;
    }
    // Janelas sem giro de verdade seguidas, contando do fim.
    int quietRun = 0;
    float runMaxDps = 0.0f;
    while (quietRun < _excCount && _exc[_excCount - 1 - quietRun].quiet) {
        runMaxDps = fmaxf(runMaxDps, _exc[_excCount - 1 - quietRun].devDps);
        quietRun++;
    }
    const ExcursionWindow *last = _exc + (_excCount - m);
    float gyMean = 0.0f, gzMean = 0.0f, spreadMean = 0.0f, winMean = 0.0f;
    for (int k = 0; k < m; k++) {
        if (!last[k].quiet) {
            return false;  // giro de verdade entre as últimas janelas
        }
        gyMean += last[k].meanGy;
        gzMean += last[k].meanGz;
        spreadMean += last[k].spreadDps;
        winMean += last[k].winS;
    }
    gyMean /= m;
    gzMean /= m;
    spreadMean /= m;
    winMean /= m;
    const float xMean = 0.5f * (m - 1);
    float sxx = 0.0f, sxGy = 0.0f, sxGz = 0.0f;
    for (int k = 0; k < m; k++) {
        float dx = k - xMean;
        sxx += dx * dx;
        sxGy += dx * (last[k].meanGy - gyMean);
        sxGz += dx * (last[k].meanGz - gzMean);
    }
    float bGy = sxGy / sxx, bGz = sxGz / sxx;  // °/s por janela
    float ss = 0.0f;
    for (int k = 0; k < m; k++) {
        float dx = k - xMean;
        float rGy = last[k].meanGy - (gyMean + bGy * dx);
        float rGz = last[k].meanGz - (gzMean + bGz * dx);
        ss += rGy * rGy + rGz * rGz;
    }
    float perS = winMean > 0.0f ? 1.0f / winMean : 0.0f;
    float limit = PAN_ZUPT_SHIFT_RESIDUAL_DPS + PAN_BIAS_RELEARN_VIBRATION_FACTOR * spreadMean +
                  PAN_ZUPT_SHIFT_WANDER_FACTOR * sqrtf(_wanderVar);
    bool smooth = sqrtf(ss / m) < limit && hypotf(bGy, bGz) * perS <= PAN_ZUPT_SHIFT_SLOPE_MAX_DPS2;
    // Forçada: muitas janelas seguidas abaixo do limiar antigo, com médias
    // ainda perto de uma reta — o zero passeando forte demais para a reta
    // lisa. Passos de giro à mão (1° a cada 2,5 s) também ficam abaixo do
    // limiar antigo por 20 s, mas as médias saltam janela a janela.
    bool forced = quietRun >= PAN_ZUPT_SHIFT_FORCE_WINDOWS && runMaxDps < PAN_ZUPT_SHIFT_FORCE_MAX_DPS &&
                  sqrtf(ss / m) < limit + PAN_ZUPT_SHIFT_FORCE_RESIDUAL_DPS;
    if (!smooth && !forced) {
        return false;
    }
    // Bias da reta no fim da última janela (a média dela é o meio).
    float tEnd = (m - 1) - xMean + 0.5f;
    float newGy = gyMean + bGy * tEnd, newGz = gzMean + bGz * tEnd;
    // O que sai de cada janela é a parte devida ao bias errado. Janela sem
    // giro é o eixo parado e sai inteira — exceto o trecho lento de um giro
    // de verdade (a aproximação final da marca, feita devagar à mão): janelas
    // sem giro coladas a uma janela de giro, que se destacam do nível do bias
    // errado, são parte dele, e delas sai só a parte do bias. Até a 1.6.15
    // elas saíam inteiras, e cada giro perdia a aproximação (~3° por giro
    // simulado, 6° somados em bancada).
    float drift[EXC_LEN];
    bool motion[EXC_LEN];
    // Bias "verdadeiro" no meio de cada janela: nas últimas m, o da reta;
    // antes delas, interpolado entre o começo da reta e o começo da excursão
    // — que veio logo depois de uma janela parada, quando o bias usado ainda
    // estava certo (dentro do limiar). Para um zero que passeia, a reta entre
    // os dois pontos conhecidos é a melhor estimativa do caminho entre eles.
    const uint32_t lastMs = _exc[_excCount - 1].ms;
    auto backS = [&](const ExcursionWindow &w) { return (lastMs - w.ms) / 1000.0f + 0.5f * w.winS; };
    const float fitBack = m * winMean;
    const float firstBack = backS(_exc[0]);
    const float fitStartGy = newGy - bGy * perS * fitBack, fitStartGz = newGz - bGz * perS * fitBack;
    for (int k = 0; k < _excCount; k++) {
        const ExcursionWindow &w = _exc[k];
        float back = backS(w), trueGy, trueGz;
        if (back <= fitBack || firstBack <= fitBack) {
            trueGy = newGy - bGy * perS * back;
            trueGz = newGz - bGz * perS * back;
        } else {
            float frac = (back - fitBack) / (firstBack - fitBack);  // 0 no começo da reta, 1 no da excursão
            trueGy = fitStartGy + (_exc[0].biasGy - fitStartGy) * frac;
            trueGz = fitStartGz + (_exc[0].biasGz - fitStartGz) * frac;
        }
        drift[k] = projectedDps(trueGy - w.biasGy, trueGz - w.biasGz, w.tiltRad) * w.winS * PAN_SCALE_CORRECTION;
        motion[k] = !w.quiet;
    }
    // Trecho lento de um giro: perto (MOTION_REACH janelas) de uma janela de
    // giro, no MESMO sentido dele e acima do nível do bias errado. A deriva
    // perto de um giro não tem sentido preferido; a aproximação da marca, sim.
    for (bool grew = true; grew;) {
        grew = false;
        for (int k = 0; k < _excCount; k++) {
            if (motion[k]) continue;
            const ExcursionWindow &w = _exc[k];
            float excess = w.deltaDeg - drift[k];
            if (fabsf(excess) <= PAN_ZUPT_SHIFT_MOTION_BAND_DPS * w.winS * PAN_SCALE_CORRECTION) continue;
            for (int j = k - PAN_ZUPT_SHIFT_MOTION_REACH; j <= k + PAN_ZUPT_SHIFT_MOTION_REACH; j++) {
                if (j < 0 || j >= _excCount || j == k || !motion[j]) continue;
                if ((excess > 0.0f) == (_exc[j].deltaDeg - drift[j] > 0.0f)) {
                    motion[k] = true;
                    grew = true;
                    break;
                }
            }
        }
    }
    // Nunca sai mais do que a janela integrou de fato no sentido da deriva:
    // no fim do curso o integrador fica preso (clampIntegrator) e a deriva
    // nem chega a entrar.
    float erase = 0.0f;
    for (int k = 0; k < _excCount; k++) {
        float delta = _exc[k].deltaDeg;
        bool partial = (delta > 0.0f) == (drift[k] > 0.0f) && fabsf(delta) < fabsf(drift[k]);
        erase += !motion[k] || partial ? delta : drift[k];
    }
    _panDeg -= erase;
    clampIntegrator();
    // Essas janelas eram paradas: o passo entre elas também mede o passeio
    // do zero — justamente o que as janelas paradas, cortadas no limiar, não
    // conseguem medir quando o passeio é maior que ele.
    for (int k = 1; k < m; k++) {
        learnWander(last[k].meanGy - last[k - 1].meanGy, last[k].meanGz - last[k - 1].meanGz);
    }
    // Em 6 janelas, metade da inclinação da reta é passeio do zero, não
    // deriva: adotá-la inteira e extrapolá-la por 30 s afastava o bias até
    // fora de qualquer limiar (simulado: -65° em 2 min com o zero passeando).
    adoptBias(newGy, newGz, clampSlope(PAN_ZUPT_SHIFT_SLOPE_GAIN * bGy * perS),
              clampSlope(PAN_ZUPT_SHIFT_SLOPE_GAIN * bGz * perS), true);
    return true;
}

float PanSensor::clampSlope(float slope) {
    if (slope > PAN_BIAS_SLOPE_MAX_DPS2) return PAN_BIAS_SLOPE_MAX_DPS2;
    if (slope < -PAN_BIAS_SLOPE_MAX_DPS2) return -PAN_BIAS_SLOPE_MAX_DPS2;
    return slope;
}

void PanSensor::clampIntegrator() {
    // Anti-windup: o integrador nunca passa da faixa mecânica. Sem isto, um
    // erro qualquer que o empurrasse para além de ±PAN_MAX_DEG (leitura
    // espúria, bias errado, a placa girada à mão além do curso) deixava a
    // leitura presa no limite: girar de volta só descontava do excesso, e o
    // valor mostrado não saía do lugar — "travado". Com o clamp, a volta
    // responde na hora.
    float lo = _offsetDeg + PAN_MIN_DEG;
    float hi = _offsetDeg + PAN_MAX_DEG;
    if (_panDeg < lo) _panDeg = lo;
    if (_panDeg > hi) _panDeg = hi;
}

void PanSensor::adoptBias(float gyDps, float gzDps, float slopeGy, float slopeGz, bool confirmed) {
    _biasGyDps = gyDps;
    _biasGzDps = gzDps;
    _biasReady = true;
    _biasConfirmed = confirmed;
    _hasPrevIntegRate = false;
    _biasSlopeGy = slopeGy;
    _biasSlopeGz = slopeGz;
    _sinceStillS = 0.0f;
    _relearnCount = 0;
    _undoCount = 0;
    _excCount = 0;
    _prevStill = false;
    _prevMoving = false;
    _tailWindows = 0;
}

void PanSensor::resetWindow(uint32_t now) {
    _windowStartMs = now;
    _windowGySumDps = 0.0f;
    _windowGzSumDps = 0.0f;
    _windowSamples = 0;
    _windowDeltaDeg = 0.0f;
    _windowPeakDps = 0.0f;
    _windowDiffSqSum = 0.0f;
    _windowDiffCount = 0;
}

float PanSensor::readPanDeg() {
    float pan = _panDeg - _offsetDeg;
    if (pan < PAN_MIN_DEG) pan = PAN_MIN_DEG;
    if (pan > PAN_MAX_DEG) pan = PAN_MAX_DEG;
    return pan;
}

PanSensor::Diagnostics PanSensor::diagnostics() const {
    return Diagnostics{
        _panDeg, _offsetDeg, _biasGyDps, _biasGzDps, _prevMeanGyDps, _prevMeanGzDps,
        _lastGyDps, _lastGzDps, _lastTiltRad * 57.29578f, _sampleCount, _i2cFailures,
        _relearnCount, static_cast<uint8_t>(_biasReady ? 1 : 0), _spikeCount,
        _mpu.recoveries(),
    };
}

float PanSensor::minPanDeg() {
    return _peaks.hasData() ? _peaks.minValue() : readPanDeg();
}

float PanSensor::maxPanDeg() {
    return _peaks.hasData() ? _peaks.maxValue() : readPanDeg();
}

void PanSensor::calibrate() {
    _offsetDeg = _panDeg;

    // Extremos guardados são relativos ao zero antigo — ver o mesmo raciocínio
    // em AngleSensor::calibrate().
    _peaks.reset();

    // Refaz também a estimativa de bias: a calibração é feita com o eixo
    // parado na posição de referência, que é exatamente a condição ideal para
    // reestabelecer o zero do giroscópio. É o que recupera o caso de o ESP32
    // ter sido ligado com o eixo em movimento.
    _biasReady = false;
    _biasConfirmed = false;
    _hasPrevMean = false;
    _relearnCount = 0;
    _undoCount = 0;
    _excCount = 0;
    _prevStill = false;
    _prevMoving = false;
    _tailWindows = 0;
    resetWindow(millis());
}
