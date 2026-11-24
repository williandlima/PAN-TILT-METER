// Pan: tranco do motor de passo (partida/parada do tilt): a placa balança numa
// ressonância mecânica, com giro oscilando no eixo de pan (média ZERO: o pan
// real não se move) e o acelerômetro sacudido. O MPU filtra internamente a
// 1 kHz (DLPF ~21 Hz, 2ª ordem aqui) e o firmware amostra a 100 Hz.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include "Arduino.h"
#include "Config.h"
#include "Mpu6050.h"
#include "PanSensor.h"
// Ver PAN_SHOCK_* em Config.h e run.sh.
uint64_t g_now_us = 0; HardwareSerial Serial;
static const double BG[3] = {1.5, 3.0, -2.0};
static std::mt19937 rng(5); static std::normal_distribution<double> nz(0.0, 1.0);
struct Biq { double b0, b1, b2, a1, a2, z1 = 0, z2 = 0; double f(double x) { double y = b0 * x + z1; z1 = b1 * x - a1 * y + z2; z2 = b2 * x - a2 * y; return y; } };
static Biq lp(double fc, double fs) { double w = tan(M_PI * fc / fs), q = M_SQRT1_2, n = 1 / (1 + w / q + w * w); Biq b; b.b0 = w * w * n; b.b1 = 2 * b.b0; b.b2 = b.b0; b.a1 = 2 * (w * w - 1) * n; b.a2 = (1 - w / q + w * w) * n; return b; }
static Biq fg[3], fa[3];
static double outG[3], outA[3];
static double shockT0 = -1, shockF = 100, shockA = 100, shockAccG = 0.5, shockPh = 0, shockTau = 0.06;
static void tick1k(double t) {  // estado interno do MPU a 1 kHz
    // POSIÇÃO oscilando e voltando a zero: θ = Θ·e^(-d/τ)·[sin(ωd+φ) - sin φ];
    // o giro é a derivada (amostrada a 1 kHz). Integral líquida = 0.
    static double prevTh = 0;
    double osc = 0, acc = 0, th = 0;
    if (shockT0 >= 0 && t >= shockT0 && t < shockT0 + 0.8) {
        double d = t - shockT0, env = exp(-d / shockTau), w = 2 * M_PI * shockF, Th = shockA / w * 180 / M_PI * M_PI / 180;
        th = Th * env * (sin(w * d + shockPh) - sin(shockPh));
        acc = shockAccG * env * sin(w * d + shockPh + 1.0);
    }
    osc = (th - prevTh) * 1000.0; prevTh = th;
    double g[3] = {BG[0] + 0.3 * osc, BG[1] + 0.2 * osc, -osc + BG[2]};
    double a[3] = {acc, 0.5 * acc, 1.0 + 0.3 * acc};
    for (int i = 0; i < 3; i++) { outG[i] = fg[i].f(g[i]); outA[i] = fa[i].f(a[i]); }
}
bool Mpu6050::begin() { return true; }
bool Mpu6050::readAccelG(float &ax, float &ay, float &az) { ax = outA[0] + 0.0023 * nz(rng); ay = outA[1] + 0.0023 * nz(rng); az = outA[2] + 0.0023 * nz(rng); return true; }
bool Mpu6050::readMotion(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    readAccelG(ax, ay, az);
    auto clip = [](double v) { return std::max(-1000.0, std::min(1000.0, v)); };
    gx = clip(outG[0] + 0.05 * nz(rng)); gy = clip(outG[1] + 0.05 * nz(rng)); gz = clip(outG[2] + 0.05 * nz(rng));
    return true; }
bool Mpu6050::setDlpfForSampleRate(uint16_t) { return true; }
bool Mpu6050::restoreDefaultDlpf() { return true; }
bool Mpu6050::writeRegister(uint8_t, uint8_t) { return true; }
int main() {
    // Grade reduzida para a CI: as combinações que mais erravam na 1.6.32.
    const double freqs[] = {30, 45, 60}, amps[] = {100, 300};
    double sum2 = 0, worst = 0; int n = 0;
    printf("=== pan: trancos do motor do PTU (balanço de média zero) ===\n");
    for (double A : amps) for (double f : freqs) {
        double w1 = 0;
        for (int k = 0; k < 4; k++) {
            for (int i = 0; i < 3; i++) { fg[i] = lp(21, 1000); fa[i] = lp(21, 1000); }
            rng.seed(k + 1); Mpu6050 mpu; PanSensor pan(mpu);
            uint32_t ms = 0; shockT0 = -1;
            auto step = [&]() { tick1k(ms / 1000.0); g_now_us = (uint64_t)ms * 1000; pan.update(); ms++; };
            for (int i = 0; i < 20000; i++) step();
            const double p0 = pan.readPanDeg();
            for (int s = 0; s < 4; s++) {
                shockT0 = ms / 1000.0 + 0.0137 * k; shockF = f; shockA = A; shockPh = 0.9 * k + s;
                for (int i = 0; i < 10000; i++) step();
            }
            const double e = pan.readPanDeg() - p0;
            sum2 += e * e; worst = std::max(worst, fabs(e)); w1 = std::max(w1, fabs(e)); n++;
        }
        printf("  %3.0f°/s a %2.0f Hz, 4 trancos: pior %.2f°\n", A, f, w1);
    }
    const double rms = sqrt(sum2 / n);
    // 1.6.32: RMS 1,15°, pior 4,4° nesta grade; 1.6.33: 0,19° e 0,48°.
    const bool ok = rms < 0.5 && worst < 1.2;
    printf("  RMS %.2f°, pior %.2f° (limites 0,5° e 1,2°)%s\n", rms, worst, ok ? "" : "  FALHOU");
    printf(ok ? "TRANCO: TUDO OK\n" : "TRANCO: HOUVE FALHA\n");
    return ok ? 0 : 1;
}
