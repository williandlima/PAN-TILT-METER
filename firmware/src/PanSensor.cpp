#include "PanSensor.h"

#include <Arduino.h>
#include <math.h>

#include "Config.h"

bool PanSensor::sampleMotion(Vec3 &gyro, Vec3 &up) {
    float ax, ay, az;
    if (!_mpu.readMotion(ax, ay, az, gyro.x, gyro.y, gyro.z)) {
        return false;
    }
    // Vertical do MESMO burst: sem defasagem entre a direção usada na
    // projeção e as taxas projetadas. O teste de plausibilidade do driver
    // garante |a| >= ACCEL_MIN_PLAUSIBLE_G.
    float n = sqrtf(ax * ax + ay * ay + az * az);
    up = {ax / n, ay / n, az / n};
    return true;
}

float PanSensor::panRateDps(Vec3 gyro, Vec3 up) const {
    // Projeção da velocidade angular sobre a vertical, escrita em coordenadas
    // do corpo (item 1 do cabeçalho de PanSensor.h), com o bias de cada eixo
    // removido ANTES de projetar (item 2).
    return projectedDps(gyro - _bias, up);
}

float PanSensor::readInstantRateDps() {
    Vec3 gyro, up;
    if (!sampleMotion(gyro, up)) {
        return _lastRateDps;  // amostra perdida: repete a última válida
    }
    _lastRateDps = panRateDps(gyro, up) * PAN_SCALE_CORRECTION;
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

// Mediana de cada componente de n <= 5 vetores.
static Vec3 medianOf(const Vec3 *v, int n) {
    float x[5], y[5], z[5];
    for (int i = 0; i < n; i++) {
        x[i] = v[i].x;
        y[i] = v[i].y;
        z[i] = v[i].z;
    }
    return {medianOf(x, n), medianOf(y, n), medianOf(z, n)};
}

bool PanSensor::despike(Vec3 &gyro, Vec3 &up) {
    for (int i = 0; i < PAN_DESPIKE_LEN - 1; i++) {
        _gyroHist[i] = _gyroHist[i + 1];
        _upHist[i] = _upHist[i + 1];
    }
    _gyroHist[PAN_DESPIKE_LEN - 1] = gyro;
    _upHist[PAN_DESPIKE_LEN - 1] = up;
    if (_histCount < PAN_DESPIKE_LEN) {
        _histCount++;
    }
    // Só com o histórico cheio a mediana rejeita até 2 amostras ruins
    // seguidas (picos vêm em dupla); antes disso, melhor esperar.
    if (_histCount < PAN_DESPIKE_LEN) {
        return false;
    }
    int first = PAN_DESPIKE_LEN - _histCount;
    Vec3 g = medianOf(_gyroHist + first, _histCount);
    Vec3 d = g - gyro;
    if (fabsf(d.x) > PAN_SPIKE_REPORT_DPS || fabsf(d.y) > PAN_SPIKE_REPORT_DPS || fabsf(d.z) > PAN_SPIKE_REPORT_DPS) {
        _spikeCount++;
    }
    gyro = g;
    // A vertical da projeção vem do MESMO quadro; um quadro de lixo que passe
    // pelo teste de plausibilidade traz também uma vertical errada.
    Vec3 u = medianOf(_upHist + first, _histCount);
    float n = norm(u);
    up = n > 0.0f ? (1.0f / n) * u : _lastUp;
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

    Vec3 gyro, up;
    if (!sampleMotion(gyro, up)) {
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
        (fabsf(gyro.x - _lastGyro.x) > PAN_GYRO_SLEW_MAX_DPS || fabsf(gyro.y - _lastGyro.y) > PAN_GYRO_SLEW_MAX_DPS ||
         fabsf(gyro.z - _lastGyro.z) > PAN_GYRO_SLEW_MAX_DPS)) {
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
    _lastGyro = gyro;
    _lastUp = up;
    _sampleCount++;
    if (!_hasLastSample) {
        // Primeira amostra: sem intervalo anterior, não há dt para integrar.
        _hasLastSample = true;
        resetWindow(now);
        despike(gyro, up);
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
    const Vec3 raw = gyro;
    if (!despike(gyro, up)) {
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
        _bias = _bias + dtS * _biasSlope;
        _sinceStillS += dtS;
    }

    if (_biasReady) {
        // Regra do trapézio: a taxa muda entre duas amostras (início e fim de
        // cada giro), e multiplicar só a taxa nova pelo intervalo inteiro
        // errava até uma amostra (~0,2° a 20°/s) por borda de movimento.
        float rate = panRateDps(gyro, up) * PAN_SCALE_CORRECTION;
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

    if (_windowSamples == 0) {
        _windowUpStart = up;
    }
    _windowSum = _windowSum + gyro;
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
        Vec3 off = raw - gyro;
        bool rawOk = fabsf(off.x) <= PAN_SPIKE_REPORT_DPS && fabsf(off.y) <= PAN_SPIKE_REPORT_DPS &&
                     fabsf(off.z) <= PAN_SPIKE_REPORT_DPS;
        if (rawOk && _hasPrevRaw) {
            Vec3 step = raw - _prevRaw;
            _windowDiffSqSum += dot(step, step);
            _windowDiffCount++;
        }
        _hasPrevRaw = rawOk;
        _prevRaw = raw;
        float peak = norm(gyro - _bias);
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

    const Vec3 mean = (1.0f / _windowSamples) * _windowSum;
    const Vec3 prevMean = _prevMean;
    bool hadPrev = _hasPrevMean;
    _prevMean = mean;
    _hasPrevMean = true;

    if (!_biasReady) {
        // Adoção: duas janelas seguidas com a mesma média. Ver "BOOT E
        // RECUPERAÇÃO" em PanSensor.h.
        if (hadPrev) {
            float spread = norm(mean - prevMean);
            if (spread < PAN_BIAS_ADOPT_COHERENCE_DPS) {
                adoptBias(0.5f * (mean + prevMean), {0.0f, 0.0f, 0.0f}, spread < PAN_ZUPT_RATE_THRESHOLD_DPS);
            }
        }
        resetWindow(now);
        return;
    }

    // A média da janela corresponde ao bias do MEIO da janela; o bias
    // corrente já foi extrapolado até o fim dela.
    float winS = (now - _windowStartMs) / 1000.0f;
    // Bias com que a janela foi integrada, no meio dela.
    const Vec3 midBias = _bias - (0.5f * winS) * _biasSlope;
    const Vec3 dev = mean - midBias;
    const float devDps = norm(dev);
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
    float signedDps = projectedDps(dev, _lastUp);
    bool stillCandidate = devDps < threshold && _windowPeakDps < PAN_ZUPT_PEAK_DPS;

    // Movimento de TILT (rotação em torno de um eixo horizontal): aparece no
    // giro, fora da vertical, e o acelerômetro vê a vertical girar junto. Não
    // é giro de pan nem bias — a projeção já o descarta na integração —, mas
    // também não pode ensinar o bias nem entrar nas retas de bias errado
    // (reaprendizado e PAN_ZUPT_SHIFT_*): até a 1.6.16 o giro do eixo X ficava
    // fora de tudo, e era o eixo do tilt na montagem prevista. A rotação vista
    // pelo acelerômetro na janela é (u1 x u0)/T; ela tem de explicar pelo
    // menos metade do que o giro mostra fora da vertical, no mesmo sentido —
    // passeio do zero não gira a vertical, e vibração de média zero deixa o
    // giro sem média.
    bool tiltMoving = false;
    if (!stillCandidate && winS > 0.0f) {
        Vec3 perp = dev - dot(dev, _lastUp) * _lastUp;
        Vec3 seen = (57.29578f / winS) * cross(_lastUp, _windowUpStart);
        float perp2 = dot(perp, perp);
        tiltMoving = perp2 > PAN_TILT_MOVE_DPS * PAN_TILT_MOVE_DPS && dot(perp, seen) > PAN_TILT_MOVE_MATCH * perp2;
    }

    if (stillCandidate && _prevMoving && _tailWindows < PAN_ZUPT_TAIL_MAX_WINDOWS &&
        isEdgeOf(signedDps, spreadDps, _lastMovingDps)) {
        // Cauda de um giro que está freando devagar (à mão): ainda é o mesmo
        // movimento — mantém o que foi integrado e não ensina o bias.
        _tailWindows++;
        _lastMovingDps = signedDps;
        pushRelearnWindow(mean, _windowDeltaDeg, spreadDps);
        pushExcursion({mean, _windowDeltaDeg, winS, _lastUp, spreadDps, devDps, midBias, now, true});
        tryShiftEscape();
        resetWindow(now);
        return;
    }

    if (stillCandidate) {
        // Janela parada: refina o bias (filtro alfa-beta: nível e velocidade
        // de deriva) e desfaz o que foi integrado nela, para o ruído do giro
        // não virar random walk enquanto parado.
        pushBiasUndo(now, winS, signedDps, spreadDps, mean);
        if (prevStill && hadPrev) {
            learnWander(mean - prevMean);
        }
        _prevStill = true;
        _excCount = 0;
        _prevMoving = false;
        _tailWindows = 0;
        _bias = _bias + PAN_ZUPT_BIAS_ALPHA * dev;
        if (winS > 0.0f) {
            _biasSlope = clampSlope(_biasSlope + (PAN_ZUPT_SLOPE_BETA / winS) * dev);
        }
        _sinceStillS = 0.0f;
        _panDeg -= _windowDeltaDeg;
        clampIntegrator();
        _biasConfirmed = true;
        _relearnCount = 0;
        _relearnTiltDeg = 0.0f;
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
        // Depois de undoRecentBiasLearning(), o bias pode ter voltado: o desvio
        // é o do bias com que a janela ficou integrada.
        const Vec3 usedBias = _bias - (0.5f * winS) * _biasSlope;
        const float usedDev = norm(mean - usedBias);
        bool quiet = usedDev < fmaxf(PAN_ZUPT_SHIFT_MAX_DPS, PAN_ZUPT_VIBRATION_FACTOR * spreadDps) &&
                     _windowPeakDps < PAN_ZUPT_PEAK_DPS;
        if (tiltMoving) {
            // Fica fora da reta do reaprendizado (sem interromper a sequência)
            // e não conta como janela "sem giro" — mas o que integrou sob um
            // bias errado também é desfeito se ele for reaprendido.
            _relearnTiltDeg += _windowDeltaDeg;
            pushExcursion({mean, _windowDeltaDeg, winS, _lastUp, spreadDps, usedDev, usedBias, now, false});
        } else {
            pushRelearnWindow(mean, _windowDeltaDeg, spreadDps);
            pushExcursion({mean, _windowDeltaDeg, winS, _lastUp, spreadDps, usedDev, usedBias, now, quiet});
            tryRelearn(winS);
            if (_excCount > 0) {  // tryRelearn() não adotou
                tryShiftEscape();
            }
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

void PanSensor::pushBiasUndo(uint32_t now, float winS, float signedDps, float spreadDps, Vec3 mean) {
    if (_undoCount == UNDO_LEN) {
        for (int i = 0; i < UNDO_LEN - 1; i++) {
            _undo[i] = _undo[i + 1];
        }
        _undoCount--;
    }
    _undo[_undoCount++] =
        BiasSnapshot{_bias, _biasSlope, now, _sinceStillS, _windowDeltaDeg, signedDps, spreadDps, winS, _lastUp, mean};
}

float PanSensor::projectedDps(Vec3 d, Vec3 up) {
    // Projeção na vertical (positivo = anti-horário visto de cima), no sentido
    // do pan do usuário (PAN_DIRECTION_SIGN). Toda conta que soma ao ângulo
    // passa por aqui, para o sentido ser um só.
    return PAN_DIRECTION_SIGN * dot(d, up);
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
    auto refAt = [&](uint32_t ms, float halfS) { return ref.bias + ((ms - ref.ms) / 1000.0f - halfS) * ref.slope; };
    float devRef[UNDO_LEN];
    for (int k = 0; k < count; k++) {
        const BiasSnapshot &w = _undo[k];
        float h = 0.5f * w.winS;
        devRef[k] = w.signedDps + projectedDps((w.bias - h * w.slope) - refAt(w.ms, h), w.up);
    }
    float h = 0.5f * winS;
    float limit = motionDps + projectedDps((_bias - h * _biasSlope) - refAt(now, h), _lastUp);

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
    auto revAt = [&](uint32_t ms, float halfS) { return s.bias + ((ms - s.ms) / 1000.0f - halfS) * s.slope; };
    float before = _panDeg;
    for (int k = revertTo; k < count; k++) {
        const BiasSnapshot &w = _undo[k];
        float hk = 0.5f * w.winS;
        // Só a cadeia volta; antes dela, só a correção de bias (a janela foi
        // cancelada, então não há o que corrigir nela).
        if (k < first) continue;
        float restored = w.deltaDeg + projectedDps((w.bias - hk * w.slope) - revAt(w.ms, hk), w.up) * w.winS *
                                          PAN_SCALE_CORRECTION;
        _panDeg += restored;
        // Voltaram a ser giro: fazem parte da excursão que começa aqui.
        pushExcursion({w.mean, restored, w.winS, w.up, w.spreadDps, fabsf(w.signedDps), revAt(w.ms, hk), w.ms, true});
    }
    _panDeg += projectedDps((_bias - h * _biasSlope) - revAt(now, h), _lastUp) * winS * PAN_SCALE_CORRECTION;
    clampIntegrator();
    _windowDeltaDeg += _panDeg - before;

    // O snapshot guarda o bias no FIM da janela `first` (antes da atualização
    // dela); daqui segue extrapolado.
    _bias = revAt(now, 0.0f);
    _biasSlope = s.slope;
    _sinceStillS = s.sinceStillS + (now - s.ms) / 1000.0f;
}

void PanSensor::pushRelearnWindow(Vec3 mean, float deltaDeg, float spreadDps) {
    _relearnMean[_relearnHead] = mean;
    _relearnDeltaDeg[_relearnHead] = deltaDeg;
    _relearnSpreadDps[_relearnHead] = spreadDps;
    _relearnHead = (_relearnHead + 1) % RELEARN_LEN;
    if (_relearnCount < RELEARN_LEN) {
        _relearnCount++;
    }
}

namespace {
// Reta de mínimos quadrados de n médias (índice 0 = a mais antiga), por eixo:
// média, inclinação (°/s por janela) e resíduo RMS (os três eixos juntos).
struct LineFit {
    Vec3 mean, slope;
    float rms;
};
template <typename At>
LineFit fitLine(int n, At at) {
    const float xMean = 0.5f * (n - 1);
    Vec3 mean{0.0f, 0.0f, 0.0f};
    for (int k = 0; k < n; k++) mean = mean + at(k);
    mean = (1.0f / n) * mean;
    float sxx = 0.0f;
    Vec3 sxy{0.0f, 0.0f, 0.0f};
    for (int k = 0; k < n; k++) {
        float dx = k - xMean;
        sxx += dx * dx;
        sxy = sxy + dx * (at(k) - mean);
    }
    Vec3 slope = (1.0f / sxx) * sxy;
    float ss = 0.0f;
    for (int k = 0; k < n; k++) {
        Vec3 r = at(k) - (mean + (k - xMean) * slope);
        ss += dot(r, r);
    }
    return {mean, slope, sqrtf(ss / n)};
}
}  // namespace

void PanSensor::tryRelearn(float winS) {
    if (_relearnCount < RELEARN_LEN) {
        return;
    }
    // Reta de mínimos quadrados das médias: bias constante ou derivando com a
    // temperatura cai sobre ela; um giro de verdade, com aceleração e
    // frenagem, não.
    const int n = RELEARN_LEN;
    LineFit fit = fitLine(n, [&](int k) { return _relearnMean[(_relearnHead + k) % RELEARN_LEN]; });
    float erase = _relearnTiltDeg;
    for (int k = 0; k < RELEARN_LEN; k++) erase += _relearnDeltaDeg[k];
    // Sob vibração as médias ondulam em torno da reta (ver
    // PAN_ZUPT_VIBRATION_FACTOR), e um limite fixo nunca seria atingido: um
    // bias errado ficaria para sempre. O limite cresce com a vibração.
    float spreadMean = 0.0f;
    for (int k = 0; k < RELEARN_LEN; k++) spreadMean += _relearnSpreadDps[k];
    spreadMean /= n;
    float limit = _biasConfirmed ? PAN_BIAS_RELEARN_RESIDUAL_DPS : PAN_BIAS_RELEARN_RESIDUAL_PROVISIONAL_DPS;
    limit = fmaxf(limit, PAN_BIAS_RELEARN_VIBRATION_FACTOR * spreadMean);
    if (fit.rms >= limit) {
        return;
    }
    // Parado sob bias errado: desfaz o que essas janelas integraram e adota
    // o bias da reta no fim da última janela (a média dela é o meio).
    _panDeg -= erase;
    clampIntegrator();
    float tEnd = (n - 1) - 0.5f * (n - 1) + 0.5f;
    float perS = winS > 0.0f ? 1.0f / winS : 0.0f;
    adoptBias(fit.mean + tEnd * fit.slope, clampSlope(perS * fit.slope), false);
}

void PanSensor::learnWander(Vec3 step) {
    _wanderVar += (dot(step, step) - _wanderVar) / PAN_ZUPT_WANDER_WINDOWS;
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
    float spreadMean = 0.0f, winMean = 0.0f;
    for (int k = 0; k < m; k++) {
        if (!last[k].quiet) {
            return false;  // giro de verdade entre as últimas janelas
        }
        spreadMean += last[k].spreadDps;
        winMean += last[k].winS;
    }
    spreadMean /= m;
    winMean /= m;
    LineFit fit = fitLine(m, [&](int k) { return last[k].mean; });
    const Vec3 b = fit.slope;  // °/s por janela
    float perS = winMean > 0.0f ? 1.0f / winMean : 0.0f;
    float limit = PAN_ZUPT_SHIFT_RESIDUAL_DPS + PAN_BIAS_RELEARN_VIBRATION_FACTOR * spreadMean +
                  PAN_ZUPT_SHIFT_WANDER_FACTOR * sqrtf(_wanderVar);
    bool smooth = fit.rms < limit && norm(b) * perS <= PAN_ZUPT_SHIFT_SLOPE_MAX_DPS2;
    // Forçada: muitas janelas seguidas abaixo do limiar antigo, com médias
    // ainda perto de uma reta — o zero passeando forte demais para a reta
    // lisa. Passos de giro à mão (1° a cada 2,5 s) também ficam abaixo do
    // limiar antigo por 20 s, mas as médias saltam janela a janela.
    bool forced = quietRun >= PAN_ZUPT_SHIFT_FORCE_WINDOWS && runMaxDps < PAN_ZUPT_SHIFT_FORCE_MAX_DPS &&
                  fit.rms < limit + PAN_ZUPT_SHIFT_FORCE_RESIDUAL_DPS;
    if (!smooth && !forced) {
        return false;
    }
    // Bias da reta no fim da última janela (a média dela é o meio).
    float tEnd = 0.5f * (m - 1) + 0.5f;
    const Vec3 newBias = fit.mean + tEnd * b;
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
    const Vec3 fitStart = newBias - (perS * fitBack) * b;
    for (int k = 0; k < _excCount; k++) {
        const ExcursionWindow &w = _exc[k];
        float back = backS(w);
        Vec3 trueBias;
        if (back <= fitBack || firstBack <= fitBack) {
            trueBias = newBias - (perS * back) * b;
        } else {
            float frac = (back - fitBack) / (firstBack - fitBack);  // 0 no começo da reta, 1 no da excursão
            trueBias = fitStart + frac * (_exc[0].bias - fitStart);
        }
        drift[k] = projectedDps(trueBias - w.bias, w.up) * w.winS * PAN_SCALE_CORRECTION;
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
        learnWander(last[k].mean - last[k - 1].mean);
    }
    // Em 6 janelas, metade da inclinação da reta é passeio do zero, não
    // deriva: adotá-la inteira e extrapolá-la por 30 s afastava o bias até
    // fora de qualquer limiar (simulado: -65° em 2 min com o zero passeando).
    adoptBias(newBias, clampSlope((PAN_ZUPT_SHIFT_SLOPE_GAIN * perS) * b), true);
    return true;
}

float PanSensor::clampSlope(float slope) {
    if (slope > PAN_BIAS_SLOPE_MAX_DPS2) return PAN_BIAS_SLOPE_MAX_DPS2;
    if (slope < -PAN_BIAS_SLOPE_MAX_DPS2) return -PAN_BIAS_SLOPE_MAX_DPS2;
    return slope;
}

Vec3 PanSensor::clampSlope(Vec3 slope) {
    return {clampSlope(slope.x), clampSlope(slope.y), clampSlope(slope.z)};
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

void PanSensor::adoptBias(Vec3 bias, Vec3 slope, bool confirmed) {
    _bias = bias;
    _biasReady = true;
    _biasConfirmed = confirmed;
    _hasPrevIntegRate = false;
    _biasSlope = slope;
    _sinceStillS = 0.0f;
    _relearnCount = 0;
    _relearnTiltDeg = 0.0f;
    _undoCount = 0;
    _excCount = 0;
    _prevStill = false;
    _prevMoving = false;
    _tailWindows = 0;
}

void PanSensor::resetWindow(uint32_t now) {
    _windowStartMs = now;
    _windowSum = {0.0f, 0.0f, 0.0f};
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
        _panDeg, _offsetDeg, _bias.y, _bias.z, _prevMean.y, _prevMean.z,
        _lastGyro.y, _lastGyro.z, atan2f(_lastUp.y, _lastUp.z) * 57.29578f, _sampleCount, _i2cFailures,
        _relearnCount, static_cast<uint8_t>(_biasReady ? 1 : 0), _spikeCount,
        _mpu.recoveries(), _bias.x, _lastGyro.x, _lastUp,
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
    _relearnTiltDeg = 0.0f;
    _undoCount = 0;
    _excCount = 0;
    _prevStill = false;
    _prevMoving = false;
    _tailWindows = 0;
    resetWindow(millis());
}
