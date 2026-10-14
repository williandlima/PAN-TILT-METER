// Simulação host do PanSensor REAL (firmware/src/PanSensor.cpp), com MPU6050
// falso: bias de fábrica do giro + mudanças de posição da placa. Ver run.sh.
#include <algorithm>
#include <cstdio>
#include <random>
#include <cmath>
#include "Arduino.h"
#include "Config.h"
#include "Mpu6050.h"
#include "PanSensor.h"

uint64_t g_now_us = 0;
HardwareSerial Serial;

// Estado físico simulado
static double g_tiltDeg = 0, g_tiltRateDps = 0, g_panRateDps = 0;
static double BGX = 1.5, BGY = 3.0, BGZ = -2.0;  // bias de fábrica do giro (°/s), corpo
static std::mt19937 rng(42);
static std::normal_distribution<double> noise(0.0, 0.05);
// Leitura espúria isolada (ruído no I2C): a cada g_spikeEveryMs, uma amostra
// de gz vem com g_spikeDps. 0 = desligado.
static uint32_t g_spikeEveryMs = 0, g_readCount = 0;
static double g_spikeDps = 0;
// Vibração do mastro (°/s por eixo, fases diferentes em gy e gz). 0 = sem.
static double g_vibDps = 0, g_vibHz = 0;

bool Mpu6050::begin() { return true; }
bool Mpu6050::readAccelG(float &ax, float &ay, float &az) {
    double t = g_tiltDeg * M_PI / 180; ax = 0; ay = sin(t); az = cos(t); return true;
}
bool Mpu6050::readMotion(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    readAccelG(ax, ay, az);
    double t = g_tiltDeg * M_PI / 180;
    // Sensibilidade do giroscópio ~3% abaixo do nominal (ver PAN_SCALE_CORRECTION
    // em firmware/src/Config.h, calibrado em bancada na 1.6.9).
    static const double PAN_SENSOR_SCALE = 1.0 / PAN_SCALE_CORRECTION;
    gx = g_tiltRateDps + BGX + noise(rng);
    double ts = g_now_us / 1e6;
    gy = -g_panRateDps * PAN_SENSOR_SCALE * sin(t) + BGY + noise(rng) + g_vibDps * sin(2 * M_PI * g_vibHz * ts + 1.3);
    gz =  g_panRateDps * PAN_SENSOR_SCALE * cos(t) + BGZ + noise(rng) + g_vibDps * sin(2 * M_PI * g_vibHz * ts + 4.1);
    // Uma leitura a cada 10 ms (ANGLE_SAMPLE_INTERVAL_MS).
    if (g_spikeEveryMs && ++g_readCount % (g_spikeEveryMs / 10) == 0) gz = g_spikeDps;
    return true;
}
bool Mpu6050::setDlpfForSampleRate(uint16_t) { return true; }
bool Mpu6050::restoreDefaultDlpf() { return true; }
bool Mpu6050::writeRegister(uint8_t, uint8_t) { return true; }

struct Seg { double t0, t1, tiltRate, panRate; };
// Giro de pan feito à mão: desloca A graus em T s a partir de t0, com
// velocidade em cosseno levantado (parte e para suave, pico 2A/T).
struct Move { double t0, T, A; };
static const Move *g_moves = nullptr;
static int g_nMoves = 0;
static double g_minPan, g_settleFrom, g_settleMin, g_settleMax;

static double run(const char *name, const Seg *segs, int n, double endS, double startTilt, const double *reportAt, int nr) {
    Mpu6050 mpu; PanSensor pan(mpu);
    g_now_us = 0; g_tiltDeg = startTilt; rng.seed(42);
    g_minPan = 1e9; g_settleMin = 1e9; g_settleMax = -1e9;
    int r = 0;
    printf("== %s ==\n", name);
    for (uint32_t ms = 0; ms <= (uint32_t)(endS * 1000); ms++) {
        double t = ms / 1000.0;
        g_tiltRateDps = 0; g_panRateDps = 0;
        for (int i = 0; i < n; i++) if (t >= segs[i].t0 && t < segs[i].t1) { g_tiltRateDps = segs[i].tiltRate; g_panRateDps = segs[i].panRate; }
        for (int i = 0; i < g_nMoves; i++) {
            const Move &m = g_moves[i];
            if (t >= m.t0 && t < m.t0 + m.T) g_panRateDps += (m.A / m.T) * (1 - cos(2 * M_PI * (t - m.t0) / m.T));
        }
        g_tiltDeg += g_tiltRateDps * 0.001;
        g_now_us = (uint64_t)ms * 1000;
        pan.update();
        double p = pan.readPanDeg();
        if (p < g_minPan) g_minPan = p;
        if (g_nMoves && t >= g_settleFrom) { g_settleMin = std::min(g_settleMin, p); g_settleMax = std::max(g_settleMax, p); }
        if (r < nr && t >= reportAt[r]) { printf("  t=%6.1fs  tilt=%5.1f°  pan=%7.2f°\n", t, g_tiltDeg, p); r++; }
    }
    return pan.readPanDeg();
}

static int failures = 0;
static void expect(const char *name, double got, double want, double tol) {
    bool ok = fabs(got - want) <= tol;
    printf("  -> %s: pan final %.2f° (esperado %.2f° ±%.2f) %s\n", name, got, want, tol, ok ? "OK" : "FALHOU");
    if (!ok) failures++;
}

int main() {
    const double rep[] = {4, 10, 20, 40, 60, 90, 120, 150, 180};
    // A: placa parada, depois muda de posição (tilt 0 -> 45°), fica parada.
    Seg a[] = {{5, 8, 15, 0}};
    expect("A", run("A: muda posicao da placa (tilt 0->45), sem girar o pan", a, 1, 180, 0, rep, 9), 0, 0.5);
    // B: gira o pan +30° (20°/s), muda de posição (tilt 0->45) e volta (45->0).
    Seg b[] = {{5, 6.5, 0, 20}, {10, 13, 15, 0}, {60, 63, -15, 0}};
    expect("B", run("B: pan +30, tilt 0->45, depois tilt volta 45->0", b, 3, 180, 0, rep, 9), 29.76, 0.5);
    // C: placa ligada já inclinada (tilt 45), muda para 0.
    Seg c[] = {{5, 8, -15, 0}};
    expect("C", run("C: liga inclinada (45) e muda para 0", c, 1, 180, 45, rep, 9), 0, 0.5);
    // D: tilt 45 fixo; pan +40 e volta -40.
    Seg d[] = {{5, 7, 0, 20}, {15, 17, 0, -20}};
    expect("D", run("D: tilt 45 fixo, pan +40 e volta -40", d, 2, 120, 45, rep, 8), 0, 0.5);
    // E: bias de fábrica grande (pior caso), tilt 0->60, pan +30, tilt 60->10.
    BGX = 12; BGY = -15; BGZ = 14;
    Seg e[] = {{5, 9, 15, 0}, {12, 13.5, 0, 20}, {30, 33.33, -15, 0}};
    expect("E", run("E: bias ±15°/s, tilt 0->60, pan +30, tilt 60->10", e, 3, 120, 0, rep, 8), 29.75, 0.5);
    // F: ligada com a placa na mão (pan mexendo nos primeiros ~2,4 s), depois
    // parada; muda de posição (tilt 0->45) e gira o pan +30.
    BGX = 1.5; BGY = 3.0; BGZ = -2.0;
    Seg f[] = {{0, 0.7, 0, 15}, {0.7, 1.6, 0, -25}, {1.6, 2.4, 0, 10}, {10, 13, 15, 0}, {20, 21.5, 0, 20}};
    expect("F", run("F: liga com a placa em movimento, tilt 0->45, pan +30", f, 5, 180, 0, rep, 9), 30, 0.5);
    // G: giro lento e constante (5°/s) exatamente durante o boot — o bias sai
    // errado; a rede de segurança tem de reaprendê-lo e desfazer a deriva.
    Seg g[] = {{0, 2.0, 0, 5}, {10, 13, 15, 0}};
    expect("G", run("G: giro constante no boot (bias errado), tilt 0->45", g, 2, 180, 0, rep, 9), 0, 0.5);
    // H: varreduras do motor a 20°/s (a mais longa, 8 s) não podem disparar
    // o reaprendizado do bias.
    Seg h[] = {{5, 9, 0, -20}, {15, 23, 0, 20}, {40, 42, 15, 0}};
    expect("H", run("H: varreduras do motor -80 -> +80, tilt 0->30", h, 3, 180, 0, rep, 9), 80, 0.5);
    // I: placa parada 10 min, com uma leitura espúria de -250°/s em gz a cada
    // ~7 s (ruído no barramento I2C). Não pode derivar.
    g_spikeEveryMs = 7000; g_spikeDps = -250; g_readCount = 0;
    const double repI[] = {60, 120, 240, 360, 480, 600};
    expect("I", run("I: parada 10 min com leituras espurias de gz", nullptr, 0, 600, 0, repI, 6), 0, 0.5);
    // J: idem, com pan +30 e mudança de posição no meio.
    Seg j[] = {{20, 21.5, 0, 20}, {100, 103, 15, 0}};
    g_readCount = 0;
    expect("J", run("J: espurias + pan +30 + tilt 0->45", j, 2, 600, 0, repI, 6), 30, 0.5);
    g_spikeEveryMs = 0;

    // K-N: giros feitos À MÃO na bancada, com partida e parada suaves. Até a
    // 1.6.13 o ZUPT tomava as janelas lentas como paradas, apagava o que
    // fora integrado nelas e ensinava a velocidade ao bias: +8° em 16 s lia
    // 0,00° com excursões negativas (-0,6°) e oscilava por ~1 min. Além do
    // valor final, exige que a leitura nunca vá para o lado contrário do giro
    // e que fique estável depois de parar (de 2 s após parar até o fim).
    auto hand = [&](const char *id, const char *name, const Move *mv, int nmv, double endS, double want, double tol,
                    bool oneWay) {
        g_moves = mv; g_nMoves = nmv;
        double stop = 0;
        for (int i = 0; i < nmv; i++) stop = std::max(stop, mv[i].t0 + mv[i].T);
        g_settleFrom = stop + 2;
        const double repK[] = {stop, stop + 2, stop + 10, endS};
        double got = run(name, nullptr, 0, endS, 0, repK, 4);
        expect(id, got, want, tol);
        bool okMin = !oneWay || g_minPan > -0.05, okSettle = g_settleMax - g_settleMin < 0.05;
        if (oneWay) printf("  -> %s: menor leitura %.2f° (> -0,05) %s\n", id, g_minPan, okMin ? "OK" : "FALHOU");
        printf("  -> %s: variação após parar %.3f° (< 0,05) %s\n", id, g_settleMax - g_settleMin, okSettle ? "OK" : "FALHOU");
        if (!okMin) failures++;
        if (!okSettle) failures++;
        g_moves = nullptr; g_nMoves = 0;
    };
    const Move k[] = {{10, 4, 8}};
    hand("K", "K: giro a mao +8 em 4 s, suave", k, 1, 60, 8, 0.1, true);
    const Move l[] = {{10, 16, 8}};
    hand("L", "L: giro a mao lento, +8 em 16 s (pico 1°/s)", l, 1, 80, 8, 0.2, true);
    const Move m[] = {{10, 1.5, 1}, {12.5, 1.5, 1}, {15, 1.5, 1}, {17.5, 1.5, 1},
                      {20, 1.5, 1}, {22.5, 1.5, 1}, {25, 1.5, 1}, {27.5, 1.5, 1}};
    hand("M", "M: passos a mao de +1 (8x, 1,5 s cada, pausa de 1 s)", m, 8, 80, 8, 0.2, true);
    const Move nn[] = {{10, 32, 8}, {60, 4, -8}};
    hand("N", "N: muito lento, +8 em 32 s, depois volta -8 em 4 s", nn, 2, 100, 0, 0.4, false);

    // O: vibração forte (2°/s) a 33,3 Hz = 100/3, onde a mediana anti-picos
    // mais retifica a vibração num bias falso. Sob vibração forte o giro à
    // mão lento se perde (o limiar sobe de propósito, ver
    // PAN_ZUPT_VIBRATION_FACTOR), mas a leitura NÃO pode disparar para fora
    // do que foi girado: na 1.6.13 ela ia a +16..+25° depois de +8°, e uma
    // deriva falsa extrapolada chegou a levá-la a -65° (PAN_BIAS_SLOPE_MAX_DPS2).
    g_vibDps = 2.0; g_vibHz = 33.3;
    const Move o[] = {{10, 4, 8}};
    g_moves = o; g_nMoves = 1;
    const double repO[] = {14, 30, 60};
    expect("O", run("O: vibracao 2°/s a 33,3 Hz + giro a mao +8 em 4 s (nao pode disparar)", nullptr, 0, 60, 0, repO, 3), 4, 4.5);
    g_moves = nullptr; g_nMoves = 0;
    g_vibDps = 0;

    printf(failures ? "PAN: HOUVE FALHA\n" : "PAN: TUDO OK\n");
    return failures ? 1 : 0;
}
