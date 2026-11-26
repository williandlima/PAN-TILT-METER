// Tilt: acomodação depois do TRANCO do motor do PTU (1.6.34). Mesmo MPU6050
// falso de tilt_sim.cpp, com um tranco na partida e na parada de cada
// movimento: aceleração linear amortecida (o acelerômetro sacudido) e um
// balanço de rotação de média zero. Bancada de 26/11/2026 contra o PRO-3600:
// lendo 3-5 s depois de parar, até 0,16° de diferença (1.6.33). Ver run.sh.
// Fusão giroscópio + acelerômetro no TILT — AngleSensor e PanSensor REAIS,
// com MPU6050 falso fisicamente coerente: o giro vê a rotação real (com bias
// de fábrica, erro de escala e ruído) e o acelerômetro vê a gravidade MAIS a
// aceleração linear (vibração do motor), que não é rotação. Ver run.sh.
// Dois AngleSensor leem o mesmo sensor: um com o giro ligado (1.6.23) e um
// só com o acelerômetro (o método até a 1.6.22), para comparar lado a lado.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <cstdlib>
#include "Arduino.h"
#include "Config.h"
#include "Mpu6050.h"
#include "AngleSensor.h"
#include "PanSensor.h"

uint64_t g_now_us = 0;
HardwareSerial Serial;

static double g_tilt = 0, g_tiltRate = 0;       // rotação REAL (°, °/s)
static double g_vibDeg = 0, g_vibHz = 0;         // aceleração linear em Y, como ângulo aparente (°)
static double g_mountDeg = 0;                    // eixo do tilt a ψ do X do chip (no plano XY)
static double g_gyroScale = 0.97;
static double g_shockT0 = -1e9, g_shockG = 0.0, g_shockF = 30, g_shockRotDeg = 0.0, g_shockTau = 0.08;                // sensibilidade do giro 3% abaixo do nominal
static int g_spikeEvery = 0, g_reads = 0;        // leitura espúria de 500°/s em gx a cada N leituras
static const double BG[3] = {1.5, 3.0, -2.0};    // bias de fábrica (°/s)
static std::mt19937 rng(9);
static std::normal_distribution<double> nz(0.0, 1.0);

// Ruído sorteado UMA vez por instante: os dois AngleSensor e o PanSensor leem
// o mesmo chip, então têm de ver a mesma amostra (senão a comparação entre os
// métodos mede o ruído, não o método).
static double g_noise[6]; static uint64_t g_noiseAt = ~0ull;
static void noise() {
    if (g_noiseAt == g_now_us) return;
    g_noiseAt = g_now_us;
    for (double &v : g_noise) v = nz(rng);
    g_reads++;
}
static void physics(double out_u[3], double out_w[3]) {
    // Como nos outros simuladores, o acelerômetro "vê" um ângulo menor que o
    // real (é o que TILT_SCALE_CORRECTION_NEG/_POS compensam); o giro vê a
    // rotação verdadeira.
    const double seen = g_tilt / (g_tilt >= 0 ? TILT_SCALE_CORRECTION_POS : TILT_SCALE_CORRECTION_NEG);
    const double D = M_PI / 180, th = -seen * D, m = g_mountDeg * D;
    const double a[3] = {cos(m), sin(m), 0};                  // eixo do tilt nos eixos do chip
    const double z[3] = {0, 0, 1};
    const double axz[3] = {a[1], -a[0], 0};                  // a × z
    const double adz = 0;                                     // a · z
    for (int i = 0; i < 3; i++) out_u[i] = z[i] * cos(th) + axz[i] * sin(th) + a[i] * adz * (1 - cos(th));
    for (int i = 0; i < 3; i++) out_w[i] = g_tiltRate * a[i];
}
bool Mpu6050::begin() { return true; }
bool Mpu6050::readAccelG(float &ax, float &ay, float &az) {
    double u[3], w[3]; physics(u, w);
    const double t = g_now_us / 1e6;
    // Aceleração linear (não gira nada): soma em Y o que desloca o ângulo aparente.
    double lin = tan(g_vibDeg * M_PI / 180) * sin(2 * M_PI * g_vibHz * t);
    double linZ = 0;
    { double d = t - g_shockT0; if (d >= 0 && d < 1.0) { double e = exp(-d / g_shockTau); lin += g_shockG * e * sin(2 * M_PI * g_shockF * d); linZ = 0.6 * g_shockG * e * sin(2 * M_PI * g_shockF * d + 1.1); } }
    noise();
    ax = u[0] + 0.0023 * g_noise[0]; ay = u[1] + lin + 0.0023 * g_noise[1]; az = u[2] + linZ + 0.0023 * g_noise[2];
    return true;
}
bool Mpu6050::readMotion(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    readAccelG(ax, ay, az);
    double u[3], w[3]; physics(u, w);
    gx = g_gyroScale * w[0] + BG[0] + 0.05 * g_noise[3];
    gy = g_gyroScale * w[1] + BG[1] + 0.05 * g_noise[4];
    gz = g_gyroScale * w[2] + BG[2] + 0.05 * g_noise[5];
    if (g_spikeEvery && g_reads % g_spikeEvery == 0) gx = 500;
    return true;
}
bool Mpu6050::setDlpfForSampleRate(uint16_t) { return true; }
bool Mpu6050::restoreDefaultDlpf() { return true; }
bool Mpu6050::writeRegister(uint8_t, uint8_t) { return true; }

static int failures = 0;
static void check(const char *name, bool ok, const char *fmt, double a = 0, double b = 0) {
    char detail[160]; snprintf(detail, sizeof detail, fmt, a, b);
    printf("  %s %s: %s\n", ok ? "OK    " : "FALHOU", name, detail);
    if (!ok) failures++;
}

struct Rig {
    Mpu6050 mpu; PanSensor pan{mpu}; AngleSensor fused{mpu}, accel{mpu};
    uint32_t ms = 0;
    double worstDiff = 0;
    Rig() { fused.attachGyro(&pan); }
    // motion(t) define g_tilt/g_tiltRate; roda `seconds`.
    template <class F> void run(double seconds, F motion) {
        for (uint32_t end = ms + (uint32_t)(seconds * 1000); ms < end; ms++) {
            motion(ms / 1000.0);
            g_now_us = (uint64_t)ms * 1000;
            fused.update(); accel.update(); pan.update();
        }
    }
    void settle(double tilt0, double seconds = 8) {
        run(seconds, [&](double) { g_tilt = tilt0; g_tiltRate = 0; });
    }
    void zero() { fused.calibrate(); accel.calibrate(); }
};
static void reset(double mount = 0) {
    g_now_us = 0; g_tilt = g_tiltRate = 0; g_vibDeg = g_vibHz = 0; g_mountDeg = mount; g_gyroScale = 0.97;
    g_spikeEvery = 0; g_reads = 0; g_noiseAt = ~0ull; rng.seed(9);
}


int main() {
    printf("=== tilt: acomodação depois do tranco do motor do PTU ===\n");
    g_shockG = 0.6; g_shockRotDeg = 0.5; g_shockF = 30;
    reset(); g_gyroScale = 1.0; Rig r; r.settle(0, 15); r.zero();
    double w3 = 0, w5 = 0;
    const double seq[][2] = {{0, 30}, {30, 0}, {0, -30}, {-30, 0}, {0, 8}, {8, -12}};
    for (auto &m : seq) {
        double from = m[0], to = m[1];
        double x = from, v = 0, dir = to > from ? 1 : -1, acc = 60, speed = 11;
        double tStop = -1, e3 = 0, e5 = 0; bool got3 = false, got5 = false;
        double t = 0; const double tBase = r.ms / 1000.0; bool stopShock = false;
        g_shockT0 = tBase;
        r.run(20, [&](double) {
            double rem = dir * (to - x);
            if (rem > 1e-6) { v = std::min({speed, v + acc * 0.001, sqrt(2 * acc * rem)}); if (v < 0.2) v = std::min(0.2, rem / 0.001); x += dir * v * 0.001; }
            else { if (tStop < 0) { tStop = t; if (!stopShock) { g_shockT0 = tBase + t; stopShock = true; } } v = 0; x = to; }
            g_tilt = x; g_tiltRate = dir * v;
            { double d = t - (g_shockT0 - tBase); if (d >= 0 && d < 1.0) { double w = 2 * M_PI * g_shockF, e = exp(-d / g_shockTau);
                g_tilt += g_shockRotDeg * e * (sin(w * d + 0.5) - sin(0.5));
                g_tiltRate += g_shockRotDeg * e * (w * cos(w * d + 0.5) - (sin(w * d + 0.5) - sin(0.5)) / g_shockTau); } }
            if (tStop >= 0 && !got3 && t - tStop >= 3) { e3 = r.fused.readAngleDeg() - to; got3 = true; }
            if (tStop >= 0 && !got5 && t - tStop >= 5) { e5 = r.fused.readAngleDeg() - to; got5 = true; }
            t += 0.001;
        });
        printf("  %+4.0f -> %+4.0f: erro 3 s depois de parar %+.2f°, 5 s %+.2f°\n", from, to, e3, e5);
        w3 = std::max(w3, fabs(e3)); w5 = std::max(w5, fabs(e5));
    }
    // 1.6.33: até 0,52° aos 3 s e 0,28° aos 5 s nesta sequência.
    check("tranco de 0,6 g: pior erro 3 s depois de parar", w3 < 0.2, "%.2f° (limite 0,20°)", w3);
    check("tranco de 0,6 g: pior erro 5 s depois de parar", w5 < 0.08, "%.2f° (limite 0,08°)", w5);
    printf(failures ? "TILT (TRANCO): HOUVE FALHA\n" : "TILT (TRANCO): TUDO OK\n");
    return failures ? 1 : 0;
}
