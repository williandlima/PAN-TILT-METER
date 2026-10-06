// Firmware REAL (main.cpp, ModbusSlave, sensores, BLE mock) em tempo real,
// com a UART0 ligada a um pseudo-terminal: o app Python abre o caminho
// impresso na 1ª linha como se fosse a porta COM do ESP32. Ver e2e_modbus.py.
//
// Emula o que a placa real faz na linha serial:
//   - RESET_ON_OPEN=1: toda vez que o app abre a porta, o ESP32 reinicia
//     (auto-reset por DTR/RTS do chip USB-serial): despeja o log de boot da
//     ROM (lixo a 9600 baud), fica ~BOOT_MS sem responder e volta com o
//     estado zerado (calibração perdida);
//   - DROP_RX/CORRUPT_RX/DROP_TX/CORRUPT_TX: probabilidade por byte de
//     perder/corromper, em cada sentido;
//   - comando "RESET" no stdin: reset espontâneo (queda de alimentação).
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <functional>
#include <new>
#include <random>
#include <string>
#include <vector>

#include "AngleSensor.h"
#include "Arduino.h"
#include "BLEMock.h"
#include "BleServer.h"
#include "Config.h"
#include "ModbusSlave.h"
#include "Mpu6050.h"
#include "PanSensor.h"
#include "VibrationCapture.h"

uint64_t g_now_us = 0;
HardwareSerial Serial;
std::vector<BLEService *> g_services;
std::function<void(BLECharacteristic *)> g_onNotify;
void setup();
void loop();
extern Mpu6050 mpu;
extern AngleSensor angleSensor;
extern PanSensor panSensor;
extern VibrationCapture vibrationCapture;
extern ModbusSlave modbusSlave;
extern BleServer bleServer;

// ---- MPU6050 falso: tilt fixo de 10° + oscilação de 0,5° a 3 Hz; pan com
// oscilação de 2°/s a 4,5 Hz; bias de fábrica no giro.
static std::mt19937 rng(11);
static std::normal_distribution<double> nz(0.0, 0.02);
static double tNow() { return g_now_us / 1e6; }
// Sensibilidade do acelerômetro abaixo do nominal (ver
// TILT_SCALE_CORRECTION_NEG/_POS em Config.h): o sensor "vê" um ângulo menor
// que o real 10°+0,5°; a correção do firmware restaura o valor verdadeiro.
// Sempre positivo aqui (10°±0,5°), então é sempre o lado POS.
static double tiltRad() { return ((10.0 + 0.5 * sin(2 * M_PI * 3.0 * tNow())) / TILT_SCALE_CORRECTION_POS) * M_PI / 180; }
bool Mpu6050::begin() { return true; }
void Mpu6050::maintain() {}
bool Mpu6050::readAccelG(float &ax, float &ay, float &az) {
    double t = tiltRad(); ax = 0; ay = sin(t); az = cos(t); return true;
}
bool Mpu6050::readMotion(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    readAccelG(ax, ay, az);
    // PAN_SENSOR_SCALE: sensibilidade do giroscópio ~3% abaixo do nominal
    // (ver PAN_SCALE_CORRECTION em Config.h, calibrado em bancada na 1.6.9).
    static const double PAN_SENSOR_SCALE = 1.0 / PAN_SCALE_CORRECTION;
    double t = tiltRad(), w = 2.0 * PAN_SENSOR_SCALE * sin(2 * M_PI * 4.5 * tNow());
    gx = nz(rng); gy = -w * sin(t) + 3.0 + nz(rng); gz = w * cos(t) - 2.0 + nz(rng);
    return true;
}
bool Mpu6050::setDlpfForSampleRate(uint16_t) { return true; }
bool Mpu6050::restoreDefaultDlpf() { return true; }
bool Mpu6050::writeRegister(uint8_t, uint8_t) { return true; }

// ---- linha serial
static int g_master = -1;
static std::deque<uint8_t> g_rx;  // PC -> ESP32, já com as falhas aplicadas
static double pDropRx, pCorruptRx, pDropTx, pCorruptTx;
static bool g_resetOnOpen;
static uint32_t g_bootMs;
static bool g_booting = false;
static uint64_t g_bootUntilUs = 0;
static bool g_slaveOpen = false;
static unsigned g_resets = 0;

static double envd(const char *k, double d) { const char *v = getenv(k); return v ? atof(v) : d; }
static bool chance(double p) { return std::uniform_real_distribution<double>(0, 1)(rng) < p; }

static int hookAvailable() { return (int)g_rx.size(); }
static int hookRead() {
    if (g_rx.empty()) return -1;
    int b = g_rx.front(); g_rx.pop_front(); return b;
}
static size_t hookWrite(const uint8_t *d, size_t n) {
    std::vector<uint8_t> out;
    for (size_t i = 0; i < n; i++) {
        if (chance(pDropTx)) continue;
        out.push_back(chance(pCorruptTx) ? (uint8_t)(d[i] ^ (1 << (rng() % 8))) : d[i]);
    }
    if (!out.empty() && g_slaveOpen) (void)!write(g_master, out.data(), out.size());
    return n;
}

static void startReset(const char *why) {
    // Reset: o firmware para, a ROM despeja o log de boot (lixo na
    // baudrate da aplicação) e o setup() roda de novo, com tudo zerado.
    g_resets++;
    g_booting = true;
    g_bootUntilUs = g_now_us + (uint64_t)g_bootMs * 1000;
    g_rx.clear();
    std::vector<uint8_t> junk(90);
    for (auto &b : junk) b = (uint8_t)rng();
    if (g_slaveOpen) (void)!write(g_master, junk.data(), junk.size());
    fprintf(stderr, "[sim] reset (%s)\n", why);
}
static void finishBoot() {
    g_booting = false;
    g_services.clear();
    mpu.~Mpu6050(); new (&mpu) Mpu6050();
    angleSensor.~AngleSensor(); new (&angleSensor) AngleSensor(mpu);
    panSensor.~PanSensor(); new (&panSensor) PanSensor(mpu);
    vibrationCapture.~VibrationCapture(); new (&vibrationCapture) VibrationCapture(angleSensor, panSensor, mpu);
    modbusSlave.~ModbusSlave(); new (&modbusSlave) ModbusSlave(angleSensor, panSensor, vibrationCapture);
    bleServer.~BleServer(); new (&bleServer) BleServer(angleSensor, panSensor, vibrationCapture);
    setup();
}

static void pumpSerial() {
    uint8_t buf[512];
    ssize_t n = read(g_master, buf, sizeof buf);
    bool open = n > 0 || (n < 0 && errno == EAGAIN);
    if (open && !g_slaveOpen) {
        g_slaveOpen = true;
        if (g_resetOnOpen) startReset("porta aberta pelo app");
    } else if (!open && g_slaveOpen) {
        g_slaveOpen = false;
    }
    if (n <= 0 || g_booting) return;  // durante o boot a UART da aplicação não existe
    for (ssize_t i = 0; i < n; i++) {
        if (chance(pDropRx)) continue;
        g_rx.push_back(chance(pCorruptRx) ? (uint8_t)(buf[i] ^ (1 << (rng() % 8))) : buf[i]);
    }
}

static void pumpStdin() {
    struct pollfd p = {0, POLLIN, 0};
    if (poll(&p, 1, 0) <= 0) return;
    char line[64];
    if (!fgets(line, sizeof line, stdin)) { exit(0); }
    if (std::string(line).rfind("RESET", 0) == 0) startReset("comando");
    if (std::string(line).rfind("STATS", 0) == 0) { printf("RESETS %u\n", g_resets); fflush(stdout); }
}

int main() {
    pDropRx = envd("DROP_RX", 0); pCorruptRx = envd("CORRUPT_RX", 0);
    pDropTx = envd("DROP_TX", 0); pCorruptTx = envd("CORRUPT_TX", 0);
    g_resetOnOpen = envd("RESET_ON_OPEN", 1) != 0;
    g_bootMs = (uint32_t)envd("BOOT_MS", 1200);

    g_master = posix_openpt(O_RDWR | O_NOCTTY);
    grantpt(g_master); unlockpt(g_master);
    struct termios t; tcgetattr(g_master, &t); cfmakeraw(&t); tcsetattr(g_master, TCSANOW, &t);
    fcntl(g_master, F_SETFL, O_NONBLOCK);
    printf("%s\n", ptsname(g_master));
    fflush(stdout);

    g_serialHooks.available = hookAvailable;
    g_serialHooks.read = hookRead;
    g_serialHooks.write = hookWrite;

    auto t0 = std::chrono::steady_clock::now();
    setup();
    while (true) {
        g_now_us = (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0).count();
        pumpStdin();
        pumpSerial();
        if (g_booting) {
            if (g_now_us >= g_bootUntilUs) finishBoot();
        } else {
            loop();
        }
        usleep(100);
    }
}
