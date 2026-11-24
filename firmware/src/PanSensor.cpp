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
    // Mesma correção por eixo do tilt (TILT_ACCEL_* em Config.h): a vertical
    // usada na projeção do pan tem de ser a mesma que o tilt enxerga. Com os
    // valores padrão (1, 0, 0) não muda nada.
    ay = (ay - TILT_ACCEL_OFFSET_Y_G) / TILT_ACCEL_GAIN_Y;
    az = az - TILT_ACCEL_OFFSET_Z_G;
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
    const float rate = panRateDps(gyro, up);
    _lastRateDps = rate * panScale(up, rate);
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
    // TRANCO (1.6.33, ver PAN_SHOCK_*): a placa balançando numa ressonância
    // mecânica (partida e parada do motor de passo do PTU) dá saltos grandes
    // entre amostras por dezenas de amostras seguidas; um quadro de lixo do
    // barramento dá 1 a 3. Detectado ANTES do limitador de salto, que é
    // justamente o que ele precisa desligar.
    // Saltos entre leituras CONSECUTIVAS (aceitas ou não), e não contra a
    // última aceita: um quadro de lixo aceito como referência fazia as
    // amostras boas seguintes parecerem saltos (soak, semente 40). Salto
    // acima de PAN_SHOCK_SLEW_MAX_DPS é lixo, não balanço filtrado: não conta.
    float readJump = 0.0f;
    if (_hasPrevRead) {
        readJump = fmaxf(fabsf(gyro.x - _prevReadGyro.x), fmaxf(fabsf(gyro.y - _prevReadGyro.y), fabsf(gyro.z - _prevReadGyro.z)));
    }
    _prevReadGyro = gyro;
    _hasPrevRead = true;
    if (_hasLastSample) {
        const bool hit = readJump > PAN_SHOCK_JUMP_DPS && readJump <= PAN_SHOCK_SLEW_MAX_DPS;
        _shockBits = static_cast<uint8_t>((_shockBits << 1) | (hit ? 1 : 0));
        uint8_t recent = _shockBits & ((1u << PAN_SHOCK_WINDOW_SAMPLES) - 1u);
        int hits = 0;
        for (; recent; recent &= recent - 1) hits++;
        if (hits >= PAN_SHOCK_MIN_HITS) {
            _shockUntilMs = now + PAN_SHOCK_HOLD_MS;
            _shockSamplesTotal++;
        }
    }
    const bool inShock = static_cast<int32_t>(_shockUntilMs - now) > 0;
    if (inShock && !_wasInShock) {
        _shockReplayPending = true;  // refaz o começo do tranco (ver shockReplay())
    }
    _wasInShock = inShock;
    // Registro cru para o replay (aceita OU descartada pelo limitador).
    if (_hasLastSample && _biasReady) {
        pushShockRec(now, gyro, up, readJump <= PAN_SHOCK_SLEW_MAX_DPS);
    }
    // Salto impossível desde a última amostra boa (ver PAN_GYRO_SLEW_MAX_DPS):
    // quadro de lixo do barramento que passou pelo teste do acelerômetro.
    // Descartado como uma falha de leitura — nem entra na mediana, que só
    // dá conta de 2 seguidos. Depois de PAN_GYRO_SLEW_MAX_REJECTS seguidos, a
    // referência é que está velha: aceita. Em tranco, o limite sobe: os saltos
    // são reais, e descartá-los subamostrava a oscilação (erro líquido).
    const float slewMax = inShock ? PAN_SHOCK_SLEW_MAX_DPS : PAN_GYRO_SLEW_MAX_DPS;
    if (_hasLastSample && now - _lastSampleMs <= 3 * ANGLE_SAMPLE_INTERVAL_MS &&
        _slewRejects < PAN_GYRO_SLEW_MAX_REJECTS &&
        (fabsf(gyro.x - _lastGyro.x) > slewMax || fabsf(gyro.y - _lastGyro.y) > slewMax ||
         fabsf(gyro.z - _lastGyro.z) > slewMax)) {
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
        _shockRecCount = 0;    // buraco: o replay do tranco não atravessa
        _windowHadGap = true;  // fora do teste de blocos (PAN_ZUPT_SUB_MOTION_DPS)
        _histCount = 0;
        _hasPrevRaw = false;
        _prevRawAxisCount = 0;
    }
    const Vec3 raw = gyro;
    if (!despike(gyro, up)) {
        _pendingDtS += dtS;
        _shockRecCount = 0;  // trecho não integrado: replay inválido
        return;
    }
    if (_pendingDtS > 0.0f) {
        // Tempo acumulado de amostras que não foram integradas: cai inteiro
        // num bloco de PAN_ZUPT_SUB_MS e não pode contar como giro
        // concentrado (achado da revisão da 1.6.32).
        _windowHadGap = true;
    }
    dtS += _pendingDtS;
    _pendingDtS = 0.0f;

    // Só integra depois que a primeira janela estabeleceu o bias: antes
    // disso, o zero-rate de fábrica (±20°/s) jogaria o ângulo longe.
    if (_biasReady && _sinceStillS < PAN_BIAS_EXTRAPOLATION_MAX_S) {
        // Deriva térmica do bias continua durante o movimento, quando não
        // há janela parada para corrigi-lo: segue a velocidade estimada.
        // Só por um tempo limitado — a estimativa envelhece.
        _bias = _bias + dtS * extrapolationSlope();
        _sinceStillS += dtS;
    }

    if (_biasReady) {
        // Regra do trapézio: a taxa muda entre duas amostras (início e fim de
        // cada giro), e multiplicar só a taxa nova pelo intervalo inteiro
        // errava até uma amostra (~0,2° a 20°/s) por borda de movimento.
        // Em tranco integra a amostra CRUA: a mediana de 5 retifica uma
        // oscilação forte num desvio de sinal fixo, que virava giro (ver
        // PAN_SHOCK_*). Fora dele, a mediana segue descartando leituras
        // espúrias.
        const float rawRate = panRateDps(inShock ? raw : gyro, up);
        float rate = rawRate * panScale(up, rawRate);
        float prevRate = _hasPrevIntegRate ? _prevIntegRateDps : rate;
        _prevIntegRateDps = rate;
        _hasPrevIntegRate = true;
        float before = _panDeg;
        _panDeg += 0.5f * (prevRate + rate) * dtS;
        clampIntegrator();
        // O que foi EFETIVAMENTE aplicado (após o clamp), para o cancelamento
        // da janela desfazer exatamente isso.
        float applied = _panDeg - before;
        if (_shockRecCount > 0) {
            ShockRec &last = _shockRec[(_shockRecHead + SHOCK_REC_LEN - 1) % SHOCK_REC_LEN];
            if (last.ms == now) {
                last.accepted = true;
                last.applied = applied;
            }
        }
        if (inShock && _shockReplayPending) {
            _shockReplayPending = false;
            const float corr = shockReplay();
            if (corr != 0.0f) {
                const float b2 = _panDeg;
                _panDeg += corr;
                clampIntegrator();
                applied += _panDeg - b2;
            }
            // A próxima borda do trapézio parte da amostra CRUA atual.
            _prevIntegRateDps = rate;
        }
        _windowDeltaDeg += applied;
        _subDeltaDeg += applied;
        _subAbsDeg += fabsf(applied);
        if (now - _subStartMs >= PAN_ZUPT_SUB_MS) {
            closeSubBlock();
            _subStartMs = now;
        }
    }

    if (_windowSamples == 0) {
        _windowUpStart = up;
    }
    accumulateAccRotation(gyro, up);
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
        // Para o teste de blocos (PAN_ZUPT_SUB_MOTION_DPS), a mesma medida com
        // a SEGUNDA diferença no eixo de pan (1.6.30): a aceleração de um giro
        // (rampa de velocidade, ~2,5°/s por amostra a 250°/s²) entra na
        // primeira diferença como "vibração" e subiria o limite justamente na
        // janela que contém a partida ou a frenagem de um passo curto. A
        // segunda diferença zera numa rampa; para ruído branco, d2²/3 tem a
        // mesma média que d1². Fora do eixo de pan segue a primeira diferença.
        // O limiar principal da janela continua com a medida acima.
        const Vec3 n = panAxis(up);
        const float rawAxis = dot(raw, n);
        if (rawOk && _hasPrevRaw) {
            Vec3 step = raw - _prevRaw;
            _windowDiffSqSum += dot(step, step);
            _windowDiffCount++;
            const float along = dot(step, n);
            const Vec3 perp = step - along * n;
            float axisSq = along * along;
            if (_prevRawAxisCount >= 2) {
                const float d2 = rawAxis - 2.0f * _prevRawAxis[1] + _prevRawAxis[0];
                axisSq = d2 * d2 / 3.0f;
            }
            _windowDiff2SqSum += dot(perp, perp) + axisSq;
        }
        if (!rawOk) {
            _prevRawAxisCount = 0;
        } else {
            _prevRawAxis[0] = _prevRawAxis[1];
            _prevRawAxis[1] = rawAxis;
            if (_prevRawAxisCount < 2) _prevRawAxisCount++;
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
        // RECUPERAÇÃO" em PanSensor.h. A média é tomada SEM a rotação que o
        // acelerômetro viu na janela (1.6.19): sob balanço lento do mastro, a
        // média de 1 s oscila mais que PAN_BIAS_ADOPT_COHERENCE_DPS de uma
        // janela para a outra, o bias nunca era adotado e o pan nunca
        // integrava (cenário V5 a 0,6 Hz: leitura parada em 0°). O balanço
        // gira a vertical junto, então o acelerômetro o vê e ele sai da conta;
        // giro de pan não gira a vertical e continua barrando a adoção.
        Vec3 bootSeen, bootMean;
        Vec3 meanC = mean;
        if (accRotation(bootSeen, bootMean)) {
            meanC = bootMean - bootSeen;
        }
        const Vec3 prevMeanC = _prevMeanCorr;
        _prevMeanCorr = meanC;
        if (hadPrev) {
            float spread = norm(meanC - prevMeanC);
            if (spread < PAN_BIAS_ADOPT_COHERENCE_DPS) {
                adoptBias(0.5f * (meanC + prevMeanC), {0.0f, 0.0f, 0.0f}, spread < PAN_ZUPT_RATE_THRESHOLD_DPS);
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
        // Para aprender o eixo do tilt vale a média INTEIRA da janela (num
        // tilt puro ela é ṫ·â), e não `perp`: tirar a componente vertical
        // forçaria â ⟂ u e apagaria justamente a inclinação do eixo de pan que
        // se quer corrigir. Janelas com giro de pan junto ficam de fora: no
        // tilt puro, |dev·u| = |ṫ|·|â·u| ≤ |ṫ|·sen(desnível).
        float along = dot(dev, _lastUp);
        if (tiltMoving && perp2 > PAN_TILT_AXIS_LEARN_DPS * PAN_TILT_AXIS_LEARN_DPS &&
            along * along < PAN_TILT_AXIS_MAX_PAN_FRACTION * PAN_TILT_AXIS_MAX_PAN_FRACTION * dot(dev, dev)) {
            Vec3 um = _windowUpStart + _lastUp;
            float un = norm(um);
            learnTiltAxis(dev, dev, un > 0.0f ? (1.0f / un) * um : _lastUp, winS);
        }
    }

    // BALANÇO DO MASTRO (1.6.19). Balanço é rotação em torno de eixos
    // horizontais: não tem componente no eixo de pan, mas deixa na média de
    // 1 s um resíduo HORIZONTAL (fração de período que sobra) que o gate de 3
    // eixos acima confunde com giro — e o estimador de vibração, que usa a
    // diferença entre amostras sucessivas, não enxerga balanço lento (1-2
    // Hz). Sob balanço, nenhuma janela ficava "parada", o bias deixava de ser
    // corrigido e derivava pela extrapolação (cenário V3b: 5° em 90 s com o
    // eixo parado). Para o PAN só importa a componente ao longo do eixo de pan:
    // se ela está abaixo do limiar, o resto é horizontal e modesto, e não há
    // tilt em curso, a janela é parada — mas o bias só aprende a componente
    // vertical, a única que o balanço não contamina.
    //
    //
    // E o bias horizontal continua sendo aprendido nessas janelas, mas da
    // forma certa: sem giro de pan, a média do giro fora do eixo de pan é o
    // erro do bias MAIS a rotação real da placa (balanço), e essa rotação o
    // acelerômetro vê — a vertical gira junto: (u_fim × u_início)/T. A
    // diferença é o erro do bias (o estimador clássico giro × acelerômetro).
    // Bias horizontal andando sem balanço: o acelerômetro vê ~0 e ele é
    // aprendido inteiro, como numa janela parada comum.
    const Vec3 nAxis = panAxis(_lastUp);
    const float vertDps = dot(dev, nAxis);
    bool swayStill = false;
    Vec3 swayLearnDev{0.0f, 0.0f, 0.0f};
    if (!stillCandidate && fabsf(vertDps) < threshold && _windowPeakDps < PAN_ZUPT_PEAK_DPS && winS > 0.0f) {
        Vec3 seen{0.0f, 0.0f, 0.0f}, subMean = mean;
        if (!accRotation(seen, subMean)) {
            seen = (57.29578f / winS) * cross(_lastUp, _windowUpStart);
        }
        seen = seen - dot(seen, nAxis) * nAxis;
        const Vec3 subDev = subMean - midBias;
        const Vec3 horiz = subDev - dot(subDev, nAxis) * nAxis;
        // Balanço lento (abaixo de ~1 Hz) também gira a vertical o bastante
        // para a janela ser reconhecida como movimento de TILT — numa janela
        // de 1 s, os dois são a mesma coisa: rotação horizontal vista pelo
        // acelerômetro. Para o pan, ambos são "pan parado"; o que separa um
        // tilt de verdade (motor, mão) é a velocidade da rotação vista. Tilt
        // rápido fica de fora: a dinâmica (aceleração centrípeta) deixa o
        // acelerômetro menos fiel, e ele não deve ensinar o bias.
        const bool realTilt = tiltMoving && norm(seen) >= PAN_SWAY_MAX_ROT_DPS;
        // O limite vale para o resíduo horizontal que o acelerômetro NÃO
        // explica (erro do bias + ruído), e não para o bruto: balanço lento e
        // forte deixa mais de 1°/s na média e cairia no caminho de movimento,
        // cuja readoção usa a média crua — contaminada pelo balanço.
        const Vec3 unexplained = horiz - seen;
        if (!realTilt && norm(seen) >= PAN_SWAY_MIN_SEEN_DPS && norm(unexplained) < PAN_SWAY_MAX_DPS) {
            stillCandidate = true;
            swayStill = true;
            swayLearnDev = vertDps * nAxis + PAN_SWAY_HORIZONTAL_GAIN * unexplained;
        }
    }

    // Giro concentrado num pedaço da janela (frenagem ou partida de um passo
    // curto): a média de 1 s o dilui abaixo do limiar. Ver
    // PAN_ZUPT_SUB_MOTION_DPS.
    closeSubBlock();
    // Janela com buraco de amostragem (loop atrasado, barramento travado)
    // fica de fora: o buraco é integrado de uma vez, com a taxa estimada nas
    // pontas, e aparece como giro concentrado num bloco sem ser — numa
    // janela parada, cancelá-lo (como até a 1.6.29) é o certo.
    if (stillCandidate && !_windowHadGap) {
        const float subS = PAN_ZUPT_SUB_MS / 1000.0f;
        float fastVar2 = _windowDiffCount > 0 ? _windowDiff2SqSum / (2.0f * _windowDiffCount) : 0.0f;
        float vibVar2 = fastVar2 - PAN_ZUPT_NOISE_SPREAD_DPS * PAN_ZUPT_NOISE_SPREAD_DPS;
        float spread2Dps = vibVar2 > 0.0f ? sqrtf(vibVar2) : 0.0f;
        const float subLimitDeg = fmaxf(PAN_ZUPT_SUB_MOTION_DPS, PAN_ZUPT_VIBRATION_FACTOR * spread2Dps) * subS;
        if (fabsf(_windowSubMaxDeg) > subLimitDeg && _windowDeltaDeg * _windowSubMaxDeg > 0.0f &&
            fabsf(_windowDeltaDeg) >= PAN_ZUPT_SUB_NET_FRACTION * fabsf(_windowSubMaxDeg) &&
            fabsf(_windowSubMaxDeg) >= PAN_ZUPT_SUB_CONCENTRATION * fabsf(_windowDeltaDeg) &&
            fabsf(_windowSubMaxDeg) >= PAN_ZUPT_SUB_COHERENCE * _windowSubMaxAbsDeg &&
            fabsf(_windowDeltaDeg) >= PAN_ZUPT_SUB_WINDOW_COHERENCE * _windowSubNetAbsSum) {
            stillCandidate = false;
            swayStill = false;
        }
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
        const Vec3 learnDev = swayStill ? swayLearnDev : dev;
        if (prevStill && hadPrev) {
            Vec3 step = mean - prevMean;
            learnWander(swayStill ? dot(step, nAxis) * nAxis : step);
        }
        _prevStill = true;
        _excCount = 0;
        _prevMoving = false;
        _tailWindows = 0;
        _bias = _bias + PAN_ZUPT_BIAS_ALPHA * learnDev;
        if (winS > 0.0f) {
            _biasSlope = clampSlope(_biasSlope + (PAN_ZUPT_SLOPE_BETA / winS) * learnDev);
        }
        _sinceStillS = 0.0f;
        _panDeg -= _windowDeltaDeg;
        clampIntegrator();
        _biasConfirmed = true;
        _lastStillMs = now;
        if (_stillSinceAdopt < 0xFFFF) _stillSinceAdopt++;
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
            tryRelearn(winS, now);
            if (_excCount > 0) {  // tryRelearn() não adotou
                tryShiftEscape();
            }
        }
    }

    resetWindow(now);
}

void PanSensor::pushShockRec(uint32_t ms, Vec3 gyroRaw, Vec3 up, bool usable) {
    const float r = panRateDps(gyroRaw, up);
    _shockRec[_shockRecHead] = ShockRec{ms, r * panScale(up, r), 0.0f, false, usable};
    _shockRecHead = (_shockRecHead + 1) % SHOCK_REC_LEN;
    if (_shockRecCount < SHOCK_REC_LEN) _shockRecCount++;
}

float PanSensor::shockReplay() {
    // Refaz com as amostras CRUAS o trecho guardado antes de o tranco ser
    // reconhecido: lá a mediana e o limitador de salto já tinham estragado a
    // integral (ver PAN_SHOCK_*). Do registro aceito mais antigo até o atual
    // (que acabou de ser integrado): trapézio das taxas cruas, inclusive das
    // amostras descartadas, menos o que foi efetivamente aplicado.
    if (_shockRecCount < 2) return 0.0f;
    const int start = (_shockRecHead + SHOCK_REC_LEN - _shockRecCount) % SHOCK_REC_LEN;
    int first = -1;
    for (int k = 0; k < _shockRecCount - 1; k++) {
        const int i = (start + k) % SHOCK_REC_LEN;
        if (_shockRec[i].accepted) { first = k; break; }
    }
    if (first < 0) return 0.0f;
    float rawInt = 0.0f, appliedSum = 0.0f;
    // Leitura com cara de lixo no trecho: não refaz nada (fica como estava).
    for (int k = first; k < _shockRecCount; k++) {
        if (!_shockRec[(start + k) % SHOCK_REC_LEN].usable) {
            _shockRecCount = 0;
            return 0.0f;
        }
    }
    // Assinatura de balanço: a taxa OSCILA em torno da média do trecho (pelo
    // menos PAN_SHOCK_MIN_SIGN_CHANGES trocas de sinal) e fica abaixo de
    // PAN_SHOCK_MAX_RATE_DPS. Coincidências de quadros de lixo e picos (soak)
    // imitavam o detector, com 1-3 valores soltos perto do fundo de escala.
    float mean = 0.0f;
    for (int k = first; k < _shockRecCount; k++) mean += _shockRec[(start + k) % SHOCK_REC_LEN].rate;
    mean /= (_shockRecCount - first);
    int changes = 0;
    float prevDev = 0.0f;
    for (int k = first; k < _shockRecCount; k++) {
        const float r = _shockRec[(start + k) % SHOCK_REC_LEN].rate;
        if (fabsf(r) > PAN_SHOCK_MAX_RATE_DPS) {
            _shockRecCount = 0;
            return 0.0f;
        }
        const float dev = r - mean;
        if (prevDev != 0.0f && dev * prevDev < 0.0f) changes++;
        if (dev != 0.0f) prevDev = dev;
    }
    if (changes < PAN_SHOCK_MIN_SIGN_CHANGES) {
        _shockRecCount = 0;
        return 0.0f;
    }
    for (int k = first + 1; k < _shockRecCount; k++) {
        const ShockRec &a = _shockRec[(start + k - 1) % SHOCK_REC_LEN];
        const ShockRec &b = _shockRec[(start + k) % SHOCK_REC_LEN];
        rawInt += 0.5f * (a.rate + b.rate) * ((b.ms - a.ms) / 1000.0f);
        appliedSum += b.applied;
    }
    _shockRecCount = 0;
    // Correção maior que a de qualquer balanço simulado (pior 2,1° a 300°/s):
    // não é balanço, fica como estava.
    const float corr = rawInt - appliedSum;
    return fabsf(corr) <= PAN_SHOCK_MAX_REPLAY_DEG ? corr : 0.0f;
}

Vec3 PanSensor::extrapolationSlope() const {
    // Zona morta suave na inclinação projetada (1.6.31, ver
    // PAN_BIAS_SLOPE_DEADBAND_DPS2): abaixo do ruído da própria estimativa
    // ela não é deriva, e projetá-la por até 30 s de giro só somava erro.
    // Acima, segue descontada da zona morta — aquecimento forte continua
    // sendo acompanhado. A inclinação guardada não muda.
    // Só em giro contínuo (mais de uma janela desde a última parada): entre
    // janelas paradas seguidas, o filtro alfa-beta tem de extrapolar com a
    // MESMA inclinação que usa para comparar (midBias); com a zona morta ali
    // (1.6.31) ele ficava neutro dentro dela, e a estimativa vagava sem
    // voltar a zero (desvio parado 0,000078 -> 0,000099°/s²).
    if (_sinceStillS < PAN_BIAS_DEADBAND_AFTER_S) {
        return _biasSlope;
    }
    auto shrink = [](float v) {
        const float d = PAN_BIAS_SLOPE_DEADBAND_DPS2;
        return v > d ? v - d : (v < -d ? v + d : 0.0f);
    };
    return {shrink(_biasSlope.x), shrink(_biasSlope.y), shrink(_biasSlope.z)};
}

void PanSensor::closeSubBlock() {
    _windowSubNetAbsSum += fabsf(_subDeltaDeg);
    if (fabsf(_subDeltaDeg) > fabsf(_windowSubMaxDeg)) {
        _windowSubMaxDeg = _subDeltaDeg;
        _windowSubMaxAbsDeg = _subAbsDeg;
    }
    _subDeltaDeg = 0.0f;
    _subAbsDeg = 0.0f;
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

float PanSensor::panScale(Vec3 up) const {
    // Escala efetiva do giro ao longo do eixo de pan (item 4 de PanSensor.h).
    // Com sensibilidade e sensibilidade cruzada diferentes por eixo, ela é a
    // forma quadrática nᵀ·M·n, que no plano do tilt vale exatamente
    // A + B·cos 2t + C·sin 2t — t é o tilt bruto, atan2(u_y, u_z), o mesmo
    // ângulo da coluna "tilt" do tools/diagnostico_pan.py.
    if (PAN_SCALE_COS2 == 0.0f && PAN_SCALE_SIN2 == 0.0f) {
        return PAN_SCALE_CORRECTION;
    }
    float t2 = 2.0f * atan2f(up.y, up.z);
    return PAN_SCALE_CORRECTION + PAN_SCALE_COS2 * cosf(t2) + PAN_SCALE_SIN2 * sinf(t2);
}

float PanSensor::panScale(Vec3 up, float rateDps) const {
    // + assimetria por sentido de giro (escala diferente no horário e no
    // anti-horário, comum em MEMS): rateDps > 0 = horário (pan do usuário).
    return panScale(up) * (1.0f + (rateDps >= 0.0f ? PAN_SCALE_ASYM : -PAN_SCALE_ASYM));
}

float PanSensor::projectedDps(Vec3 d, Vec3 up) const {
    // Projeção no eixo de pan (positivo = anti-horário visto de cima), no
    // sentido do pan do usuário (PAN_DIRECTION_SIGN). Toda conta que soma ao
    // ângulo passa por aqui, para o sentido e o eixo serem um só.
    return PAN_DIRECTION_SIGN * dot(d, panAxis(up));
}

Vec3 PanSensor::panAxis(Vec3 up) const {
    // Vertical sem a componente do eixo do tilt (item 5 de PanSensor.h). Com o
    // eixo de pan na vertical, u já é perpendicular ao eixo do tilt e isto
    // devolve u. Com o eixo de pan inclinado de β (viatura fora do nível), u
    // tem componente sen β ao longo do eixo do tilt, e projetar em u somava
    // ao pan até sen β de todo movimento de tilt.
    if (!_tiltAxisKnown) {
        return up;
    }
    Vec3 v = up - dot(up, _tiltAxis) * _tiltAxis;
    float n = norm(v);
    // Mais de ~25° entre a vertical e o plano do pan não é viatura fora do
    // nível: é estimativa ruim. Fica com a projeção clássica.
    if (n < PAN_TILT_AXIS_MIN_COS) {
        return up;
    }
    return (1.0f / n) * v;
}

void PanSensor::learnTiltAxis(Vec3 p, Vec3 dev, Vec3 um, float winS) {
    // Soma de p·pᵀ·T: o movimento de tilt é uma rotação em torno de UM eixo
    // fixo na placa, então essa matriz tem um autovetor dominante — ele.
    // Médias de janela (e não amostras): a vibração do mastro, de média zero,
    // não entra; e só janelas em que o acelerômetro confirmou a vertical
    // girando chegam aqui.
    _tiltAxisAcc[0] += p.x * p.x * winS;
    _tiltAxisAcc[1] += p.y * p.y * winS;
    _tiltAxisAcc[2] += p.z * p.z * winS;
    _tiltAxisAcc[3] += p.x * p.y * winS;
    _tiltAxisAcc[4] += p.x * p.z * winS;
    _tiltAxisAcc[5] += p.y * p.z * winS;
    _tiltAxisSeenDeg += norm(p) * winS;
    if (!_tiltAxisKnown) {
        const float d[3] = {dev.x, dev.y, dev.z}, u[3] = {um.x, um.y, um.z};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                _tiltLeakAcc[3 * i + j] += winS * d[i] * u[j];
            }
        }
    }
    const float *m = _tiltAxisAcc;
    float trace = m[0] + m[1] + m[2];
    if (trace <= 0.0f) {
        return;
    }
    // Autovetor principal por iteração de potência (3x3, converge rápido a
    // partir da estimativa anterior).
    Vec3 v = _tiltAxisKnown ? _tiltAxis : Vec3{m[0] >= m[1] && m[0] >= m[2] ? 1.0f : 0.0f,
                                               m[1] > m[0] && m[1] >= m[2] ? 1.0f : 0.0f,
                                               m[2] > m[0] && m[2] > m[1] ? 1.0f : 0.0f};
    float lambda = 0.0f;
    for (int i = 0; i < 30; i++) {
        Vec3 w{m[0] * v.x + m[3] * v.y + m[4] * v.z, m[3] * v.x + m[1] * v.y + m[5] * v.z,
               m[4] * v.x + m[5] * v.y + m[2] * v.z};
        float n = norm(w);
        if (n <= 0.0f) {
            return;
        }
        lambda = n;
        v = (1.0f / n) * w;
    }
    // Só passa a valer com movimento de tilt suficiente e um eixo claramente
    // dominante (um único eixo de rotação). Senão, continua a projeção
    // clássica na vertical, exatamente como antes.
    if (_tiltAxisSeenDeg >= PAN_TILT_AXIS_MIN_DEG && lambda >= PAN_TILT_AXIS_DOMINANCE * trace) {
        bool first = !_tiltAxisKnown;
        _tiltAxis = v;
        _tiltAxisKnown = true;
        if (first) {
            // As janelas de tilt anteriores foram integradas projetando em u;
            // o certo era em n ≈ u − (u·â)â. A diferença, somada, é
            // −Σ T·(dev·â)(u·â) = −âᵀ·(Σ T·dev·uᵀ)·â.
            const float a[3] = {v.x, v.y, v.z};
            float leak = 0.0f;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    leak += a[i] * _tiltLeakAcc[3 * i + j] * a[j];
                }
            }
            _panDeg -= PAN_DIRECTION_SIGN * leak * PAN_SCALE_CORRECTION;
            clampIntegrator();
        }
    }
    // Mantém a soma numa faixa segura para float, sem mudar a direção.
    if (trace > 1.0e6f) {
        for (int i = 0; i < 6; i++) {
            _tiltAxisAcc[i] *= 0.5f;
        }
    }
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
                                          panScale(w.up);
        _panDeg += restored;
        // Voltaram a ser giro: fazem parte da excursão que começa aqui.
        pushExcursion({w.mean, restored, w.winS, w.up, w.spreadDps, fabsf(w.signedDps), revAt(w.ms, hk), w.ms, true});
    }
    _panDeg += projectedDps((_bias - h * _biasSlope) - revAt(now, h), _lastUp) * winS * panScale(_lastUp);
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

void PanSensor::tryRelearn(float winS, uint32_t now) {
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
    float tEnd = (n - 1) - 0.5f * (n - 1) + 0.5f;
    const Vec3 newBias = fit.mean + tEnd * fit.slope;
    if (_stillSinceAdopt >= PAN_BIAS_TRUST_STILL_WINDOWS) {
        // Bias confirmado por janelas paradas só muda por deriva: uma troca
        // maior que a deriva possível desde a última janela parada é giro
        // contínuo, não bias errado (ver PAN_BIAS_RELEARN_MAX_JUMP_DPS).
        const float ageS = (now - _lastStillMs) / 1000.0f;
        const float allowed = PAN_BIAS_RELEARN_MAX_JUMP_DPS + PAN_BIAS_SLOPE_MAX_DPS2 * ageS;
        if (fabsf(projectedDps(newBias - _bias, _lastUp)) > allowed) {
            return;
        }
    }
    // Parado sob bias errado: desfaz o que essas janelas integraram e adota
    // o bias da reta no fim da última janela (a média dela é o meio).
    _panDeg -= erase;
    clampIntegrator();
    float perS = winS > 0.0f ? 1.0f / winS : 0.0f;
    adoptBias(newBias, clampSlope(perS * fit.slope), false);
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
        drift[k] = projectedDps(trueBias - w.bias, w.up) * w.winS * panScale(w.up);
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
            if (fabsf(excess) <= PAN_ZUPT_SHIFT_MOTION_BAND_DPS * w.winS * panScale(w.up)) continue;
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
    _stillSinceAdopt = 0;
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
    _windowDiff2SqSum = 0.0f;
    _subStartMs = now;
    _subDeltaDeg = 0.0f;
    _subAbsDeg = 0.0f;
    _windowSubMaxDeg = 0.0f;
    _windowSubMaxAbsDeg = 0.0f;
    _windowSubNetAbsSum = 0.0f;
    _windowHadGap = false;
    _upHeadSum = {0.0f, 0.0f, 0.0f};
    _gyroHeadHalf = {0.0f, 0.0f, 0.0f};
}

void PanSensor::accumulateAccRotation(Vec3 gyro, Vec3 up) {
    // Chamada ANTES de _windowSamples++: k é o índice desta amostra.
    const int k = _windowSamples;
    if (k < ACC_BLOCK) {
        _upHeadSum = _upHeadSum + up;
    }
    if (k < ACC_BLOCK / 2) {
        _gyroHeadHalf = _gyroHeadHalf + gyro;
    }
    _upTail[k % ACC_BLOCK] = up;
    _gyroTail[k % (ACC_BLOCK / 2)] = gyro;
}

bool PanSensor::accRotation(Vec3 &seen, Vec3 &gyroMean) const {
    const int n = _windowSamples;
    if (n < 3 * ACC_BLOCK) {
        return false;
    }
    Vec3 tail{0.0f, 0.0f, 0.0f}, gyroTail{0.0f, 0.0f, 0.0f};
    for (int i = 0; i < ACC_BLOCK; i++) {
        tail = tail + _upTail[i];
    }
    for (int i = 0; i < ACC_BLOCK / 2; i++) {
        gyroTail = gyroTail + _gyroTail[i];
    }
    float nh = norm(_upHeadSum), nt = norm(tail);
    if (nh <= 0.0f || nt <= 0.0f) {
        return false;
    }
    // Os centros dos blocos estão a ACC_BLOCK/2 amostras de cada ponta; entre
    // eles ficam n − ACC_BLOCK intervalos de amostra.
    const float subS = (_lastSampleMs - _windowStartMs) / 1000.0f * static_cast<float>(n - ACC_BLOCK) / n;
    if (subS <= 0.0f) {
        return false;
    }
    seen = (57.29578f / subS) * cross((1.0f / nt) * tail, (1.0f / nh) * _upHeadSum);
    gyroMean = (1.0f / (n - ACC_BLOCK)) * (_windowSum - _gyroHeadHalf - gyroTail);
    return true;
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
