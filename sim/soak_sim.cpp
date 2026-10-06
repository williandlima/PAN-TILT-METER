// Teste de resistência (soak) do firmware REAL — main.cpp, Mpu6050.cpp (o
// driver de verdade), AngleSensor, PanSensor, BLE/Modbus — contra um MPU6050
// emulado no nível de registrador I2C, com verdade física conhecida e
// injeção de falhas do mundo real. Ver run.sh.
//
// Cada semente roda SOAK_MINUTES minutos simulados de uma sequência
// aleatória de movimentos de pan (inclusive além do curso de ±90°) e de tilt,
// com uma calibração no meio, e em cima disso:
//   - bias de fábrica do giro (±20°/s) com deriva térmica (±3°/s);
//   - vibração de média zero e ruído;
//   - NACK e leitura curta no I2C;
//   - quadros inteiros de lixo, quadros 0xFF, picos isolados no giro;
//   - barramento travado que só volta com reset do periférico I2C;
//   - reset do MPU6050 (queda de alimentação): volta em SLEEP, lendo zeros,
//     com a configuração de fábrica;
//   - loop() travado por dezenas/centenas de ms.
// No fim de cada período parado a leitura é comparada com a verdade.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <functional>
#include <random>
#include <vector>

#include "Arduino.h"
#include "AngleSensor.h"
#include "BLEMock.h"
#include "Config.h"
#include "Mpu6050.h"
#include "PanSensor.h"
#include "Wire.h"

uint64_t g_now_us = 0;
HardwareSerial Serial;
std::vector<BLEService *> g_services;
std::function<void(BLECharacteristic *)> g_onNotify;
void setup();
void loop();
extern Mpu6050 mpu;
extern AngleSensor angleSensor;
extern PanSensor panSensor;

static const double SOAK_MINUTES = 30;
static const double PAN_TOL_DEG = 1.0, TILT_TOL_DEG = 0.3;

static std::mt19937 rng;
// Tipos de falha ligados (bits; SOAK_FAULTS no ambiente, padrão = todos):
// 1 picos no giro, 2 quadros de lixo, 4 quadros 0xFF, 8 reset do MPU,
// 16 barramento sem resposta, 32 barramento travado, 64 loop parado,
// 128 NACK/leitura curta.
static unsigned g_faultMask = 0xFF;
static bool on(unsigned bit) { return (g_faultMask & bit) != 0; }
static double urand(double a = 0, double b = 1) { return std::uniform_real_distribution<double>(a, b)(rng); }
static bool chance(double perSecond, double dtS) { return urand() < perSecond * dtS; }
static double nowS() { return g_now_us / 1e6; }

// ------------------------------------------------------------ verdade física
// tiltRateF/panRateF: taxas como o sensor as vê, depois do DLPF de 21 Hz do
// chip (1ª ordem, tau = 7,6 ms) — o chip nunca entrega um degrau de taxa.
struct Truth { double tilt = 0, pan = 0, tiltRate = 0, panRate = 0, tiltRateF = 0, panRateF = 0; } T;
struct Env { double b0[3], drift[3], tau, vibAmp, vibHz, phase[3], noise; } E;
static std::normal_distribution<double> gauss(0.0, 1.0);

// -------------------------------------------------- MPU6050 emulado (I2C)
static uint8_t regs[128];
static uint8_t regPtr;
static std::vector<uint8_t> txBuf, rxBuf;
static size_t rxPos;
static uint8_t txAddr;

struct Faults {
    double hangUntil = 0;     // barramento sem resposta até este instante
    bool stuck = false;       // só volta com Wire.end()/begin()
    int garbageLeft = 0, ffLeft = 0, gyroSpikeLeft = 0;
    int resets = 0, hangs = 0, stucks = 0, garbage = 0, ff = 0, spikes = 0, stalls = 0, busResets = 0;
} F;

static void mpuPowerOnReset() {
    memset(regs, 0, sizeof regs);
    regs[0x6B] = 0x40;  // SLEEP
    regs[0x75] = 0x68;  // WHO_AM_I
}
static bool busDown() { return F.stuck || nowS() < F.hangUntil; }

static int16_t toRaw(double v, double scale) {
    double r = std::round(v * scale);
    return (int16_t)std::max(-32767.0, std::min(32767.0, r));
}
static void dataFrame(uint8_t out[14]) {
    memset(out, 0, 14);
    if (regs[0x6B] & 0x40) return;  // dormindo: registradores de dado zerados
    // Sensibilidade do acelerômetro abaixo do nominal entre os eixos Y/Z
    // usados no atan2, assimétrica entre inclinação positiva e negativa (ver
    // TILT_SCALE_CORRECTION_NEG/_POS em firmware/src/Config.h): o sensor "vê"
    // um ângulo th menor que o real T.tilt. Isso entra tanto no atan2 do
    // tilt quanto na projeção do giro do pan logo abaixo — o PanSensor
    // recalcula seu próprio atan2 a partir do MESMO acelerômetro, sem a
    // correção, que só existe do lado do consumidor em AngleSensor.
    double t = nowS();
    double tiltScale = T.tilt >= 0 ? TILT_SCALE_CORRECTION_POS : TILT_SCALE_CORRECTION_NEG;
    double th = (T.tilt / tiltScale) * M_PI / 180;
    double g[3];
    for (int i = 0; i < 3; i++) {
        g[i] = E.b0[i] + E.drift[i] * (1 - exp(-t / E.tau)) +
               E.vibAmp * sin(2 * M_PI * E.vibHz * t + E.phase[i]) + E.noise * gauss(rng);
    }
    g[0] += T.tiltRateF;
    // Sensibilidade do giroscópio ~3% abaixo do nominal, independente da do
    // acelerômetro acima (ver PAN_SCALE_CORRECTION em Config.h, calibrado em
    // bancada na 1.6.9).
    static const double PAN_SENSOR_SCALE = 1.0 / PAN_SCALE_CORRECTION;
    g[1] += -T.panRateF * PAN_SENSOR_SCALE * sin(th);
    g[2] += T.panRateF * PAN_SENSOR_SCALE * cos(th);
    int16_t raw[7] = {
        0, toRaw(sin(th) + 0.002 * gauss(rng), 16384), toRaw(cos(th) + 0.002 * gauss(rng), 16384), 0,
        toRaw(g[0], 131), toRaw(g[1], 131), toRaw(g[2], 131),
    };
    if (F.gyroSpikeLeft > 0) {
        F.gyroSpikeLeft--;
        int axis = 1 + (int)urand(0, 2);
        raw[4 + axis] = toRaw((urand() < 0.7 ? -1 : 1) * urand(100, 245), 131);
    }
    for (int i = 0; i < 7; i++) { out[2 * i] = (uint8_t)(raw[i] >> 8); out[2 * i + 1] = (uint8_t)raw[i]; }
    if (F.garbageLeft > 0) { F.garbageLeft--; for (int i = 0; i < 14; i++) out[i] = (uint8_t)urand(0, 256); }
    if (F.ffLeft > 0) { F.ffLeft--; memset(out, 0xFF, 14); }
}

TwoWire Wire;
bool TwoWire::begin(int, int, uint32_t) { return true; }
void TwoWire::setClock(uint32_t) {}
bool TwoWire::end() { if (F.stuck) { F.stuck = false; } F.busResets++; return true; }
void TwoWire::beginTransmission(uint8_t a) { txAddr = a; txBuf.clear(); }
size_t TwoWire::write(uint8_t b) { txBuf.push_back(b); return 1; }
uint8_t TwoWire::endTransmission(bool) {
    if (txAddr != 0x68 || busDown() || (on(128) && urand() < 0.001)) return 2;  // NACK
    if (txBuf.empty()) return 0;
    regPtr = txBuf[0];
    for (size_t i = 1; i < txBuf.size(); i++) {
        uint8_t r = regPtr++;
        if (r == 0x6B && (txBuf[i] & 0x80)) { mpuPowerOnReset(); continue; }
        regs[r & 0x7F] = txBuf[i];
    }
    return 0;
}
uint8_t TwoWire::requestFrom(uint8_t a, uint8_t n) {
    rxBuf.clear(); rxPos = 0;
    if (a != 0x68 || busDown()) return 0;
    uint8_t frame[14];
    bool isData = regPtr >= 0x3B && regPtr <= 0x48;
    if (isData) dataFrame(frame);
    for (uint8_t i = 0; i < n; i++) {
        uint8_t r = regPtr + i;
        rxBuf.push_back(r >= 0x3B && r <= 0x48 ? frame[r - 0x3B] : regs[r & 0x7F]);
    }
    if (on(128) && urand() < 0.0005 && n > 1) { rxBuf.pop_back(); return (uint8_t)rxBuf.size(); }  // leitura curta
    return n;
}
int TwoWire::read() { return rxPos < rxBuf.size() ? rxBuf[rxPos++] : -1; }

// ------------------------------------------------------------- roteiro
struct Result { double maxPanErr = 0, maxTiltErr = 0; int checks = 0, fails = 0; };

static Result runSeed(unsigned seed, bool verbose) {
    rng.seed(seed);
    gauss.reset();  // normal_distribution guarda um valor entre chamadas
    g_now_us = 0; T = Truth(); F = Faults();
    for (int i = 0; i < 3; i++) {
        E.b0[i] = urand(-20, 20); E.drift[i] = urand(-3, 3); E.phase[i] = urand(0, 6.28);
    }
    E.tau = 300; E.vibAmp = urand(0, 2); E.vibHz = urand(5, 15); E.noise = 0.05;
    if (getenv("SOAK_NODRIFT")) for (double &d : E.drift) d = 0;
    if (getenv("SOAK_NOVIB")) E.vibAmp = 0;
    mpuPowerOnReset();
    g_services.clear();

    // Objetos globais do firmware (main.cpp) são construídos uma vez só; o
    // estado é reiniciado recriando-os no lugar.
    mpu.~Mpu6050(); new (&mpu) Mpu6050();
    angleSensor.~AngleSensor(); new (&angleSensor) AngleSensor(mpu);
    panSensor.~PanSensor(); new (&panSensor) PanSensor(mpu);
    setup();

    Result res;
    double expPan = 0, tiltCal = 0;
    bool calibrated = false;
    const double dt = 0.001;
    double end = SOAK_MINUTES * 60;
    double phaseEnd = 5;  // parado no boot
    double panTarget = 0, tiltTarget = 0;
    bool moving = false;
    double stoppedAt = 0;
    double stallUntil = 0;

    auto checkpoint = [&]() {
        double pan = panSensor.readPanDeg();
        double tilt = angleSensor.readAngleDeg();
        double ePan = fabs(pan - expPan), eTilt = fabs(tilt - (T.tilt - tiltCal));
        res.maxPanErr = std::max(res.maxPanErr, ePan);
        res.maxTiltErr = std::max(res.maxTiltErr, eTilt);
        res.checks++;
        bool bad = ePan > PAN_TOL_DEG || eTilt > TILT_TOL_DEG;
        if (bad) res.fails++;
        if (verbose || bad) {
            PanSensor::Diagnostics d = panSensor.diagnostics();
            double tb = 1 - exp(-nowS() / E.tau);
            printf("    t=%7.1fs pan=%8.2f (esperado %8.2f)  tilt=%6.2f (esperado %6.2f)%s\n", nowS(), pan, expPan,
                   tilt, T.tilt - tiltCal, bad ? "  <-- FALHOU" : "");
            if (bad && getenv("SOAK_DEBUG")) {
                printf("        bias gy %.3f (real %.3f) gz %.3f (real %.3f) pronto=%d fora=%u espúrias=%u reconfig=%u\n",
                       d.biasGyDps, E.b0[1] + E.drift[1] * tb, d.biasGzDps, E.b0[2] + E.drift[2] * tb, d.biasReady,
                       (unsigned)d.mismatchWindows, (unsigned)d.spikes, (unsigned)mpu.recoveries());
            }
        }
    };

    while (nowS() < end) {
        double t = nowS();
        // --- roteiro de movimento
        if (!moving && t >= phaseEnd) {
            checkpoint();
            if (!calibrated && t > end / 2) {
                // Calibrar pelo mesmo caminho do botão dos apps (BleServer::update).
                angleSensor.calibrate();
                panSensor.calibrate();
                expPan = 0; tiltCal = T.tilt; calibrated = true;
                // Premissa do firmware: parado ~2 s após calibrar (PanSensor.h).
                phaseEnd = t + 5;
                continue;
            }
            double r = urand();
            panTarget = T.pan; tiltTarget = T.tilt;
            if (r < 0.6 || r >= 0.9) {
                // Alvo relativo ao zero corrente; 20% das vezes além do curso.
                double rel = urand() < 0.2 ? (urand() < 0.5 ? -1 : 1) * urand(95, 125) : urand(-80, 80);
                panTarget = T.pan + (rel - expPan);
            }
            if (r >= 0.6) tiltTarget = urand(-25, 25);
            if (fabs(panTarget - T.pan) < 3) panTarget = T.pan;
            T.panRate = panTarget > T.pan ? 20 : (panTarget < T.pan ? -20 : 0);
            T.tiltRate = tiltTarget > T.tilt ? 15 : (tiltTarget < T.tilt ? -15 : 0);
            moving = T.panRate != 0 || T.tiltRate != 0;
            if (!moving) { phaseEnd = t + urand(8, 40); stoppedAt = t; }
        }
        if (moving) {
            if (T.panRate != 0 && (T.panRate > 0 ? T.pan >= panTarget : T.pan <= panTarget)) T.panRate = 0;
            if (T.tiltRate != 0 && (T.tiltRate > 0 ? T.tilt >= tiltTarget : T.tilt <= tiltTarget)) T.tiltRate = 0;
            if (T.panRate == 0 && T.tiltRate == 0) { moving = false; phaseEnd = t + urand(8, 40); stoppedAt = t; }
        }
        const double kLp = dt / (0.0076 + dt);
        T.panRateF += kLp * (T.panRate - T.panRateF);
        T.tiltRateF += kLp * (T.tiltRate - T.tiltRateF);
        double dPan = T.panRate * dt;
        T.pan += dPan; T.tilt += T.tiltRate * dt;
        expPan = std::max((double)PAN_MIN_DEG, std::min((double)PAN_MAX_DEG, expPan + dPan));

        // --- falhas
        bool still = !moving;
        // Buracos longos (loop parado, barramento fora) só com o eixo
        // assentado: parado há 1 s e sem giro começando em menos de 1 s. Um
        // buraco longo exatamente na borda de um giro não tem como ser
        // recuperado sem a FIFO do MPU6050 — o instante em que a taxa mudou
        // não foi amostrado. Bordas são cobertas pelos buracos curtos
        // (loop parado até 80 ms) durante o movimento.
        bool settled = still && t - stoppedAt > 1.0 && phaseEnd - t > 1.0;
        if (on(1) && chance(1.0 / 5, dt)) { F.gyroSpikeLeft = urand() < 0.3 ? 2 : 1; F.spikes++; }
        if (on(2) && chance(1.0 / 20, dt)) { F.garbageLeft = 1 + (int)urand(0, 3); F.garbage++; }
        if (on(4) && chance(1.0 / 30, dt)) { F.ffLeft = 1 + (int)urand(0, 5); F.ff++; }
        if (on(8) && chance(1.0 / 180, dt)) { mpuPowerOnReset(); F.resets++; }
        if (on(16) && settled && chance(1.0 / 120, dt)) { F.hangUntil = t + urand(0.05, 0.4); F.hangs++; }
        if (on(32) && settled && chance(1.0 / 300, dt)) { F.stuck = true; F.stucks++; }
        if (t >= stallUntil) {
            if (on(64) && settled && chance(1.0 / 60, dt)) { stallUntil = t + urand(0.05, 0.4); F.stalls++; }
            else if (on(64) && !still && chance(1.0 / 10, dt)) { stallUntil = t + urand(0.01, 0.08); F.stalls++; }
        }

        if (t >= stallUntil) loop();
        static double traceFrom = getenv("SOAK_TRACE") ? atof(getenv("SOAK_TRACE")) : -1;
        if (traceFrom >= 0 && t >= traceFrom && t < traceFrom + 3 && ((int)(t * 1000) % 20) == 0) {
            PanSensor::Diagnostics d = panSensor.diagnostics();
            printf("      tr t=%8.3f pan=%8.3f exp=%8.3f rate=%6.1f regPWR=%02x rec=%u fails=%u gz=%7.2f\n", t,
                   panSensor.readPanDeg(), expPan, T.panRate, regs[0x6B], (unsigned)mpu.recoveries(),
                   (unsigned)mpu.readFailures(), d.gzDps);
        }
        g_now_us += 1000;
    }
    if (!moving && nowS() - stoppedAt >= 5) checkpoint();  // o fim pode cair no meio de um giro

    bool cfgOk = regs[0x6B] == 0x00 && (regs[0x1A] & 7) == 4 && regs[0x1B] == 0 && regs[0x1C] == 0 && !F.stuck;
    PanSensor::Diagnostics d = panSensor.diagnostics();
    printf("  semente %2u: %3d checagens, erro máx pan %.3f° tilt %.3f° | falhas injetadas: %d resets MPU, "
           "%d barr. travado, %d hangs, %d lixo, %d 0xFF, %d picos, %d loop parado | firmware: %u reconfig., "
           "%u espúrias, %u leituras descartadas | registradores %s  %s\n",
           seed, res.checks, res.maxPanErr, res.maxTiltErr, F.resets, F.stucks, F.hangs, F.garbage, F.ff, F.spikes,
           F.stalls, (unsigned)mpu.recoveries(), (unsigned)d.spikes, (unsigned)mpu.readFailures(),
           cfgOk ? "OK" : "ERRADOS", res.fails == 0 && cfgOk ? "OK" : "FALHOU");
    if (!cfgOk) res.fails++;
    return res;
}

int main(int argc, char **argv) {
    int seeds = argc > 1 ? atoi(argv[1]) : 10;
    if (getenv("SOAK_FAULTS")) g_faultMask = (unsigned)strtoul(getenv("SOAK_FAULTS"), nullptr, 0);
    printf("== Soak: %d sementes x %.0f min simulados, firmware real com falhas injetadas ==\n", seeds, SOAK_MINUTES);
    int fails = 0;
    double worstPan = 0, worstTilt = 0;
    int first = argc > 2 ? atoi(argv[2]) : 1;
    for (int s = first; s < first + seeds; s++) {
        Result r = runSeed(s, getenv("SOAK_VERBOSE") != nullptr);
        fails += r.fails;
        worstPan = std::max(worstPan, r.maxPanErr);
        worstTilt = std::max(worstTilt, r.maxTiltErr);
    }
    printf("pior erro: pan %.3f° (tol. %.1f°), tilt %.3f° (tol. %.1f°)\n", worstPan, PAN_TOL_DEG, worstTilt, TILT_TOL_DEG);
    printf(fails ? "SOAK: HOUVE FALHA\n" : "SOAK: TUDO OK\n");
    return fails ? 1 : 0;
}
