#include "Mpu6050.h"

#include <Arduino.h>
#include <Wire.h>

#include "Config.h"

bool Mpu6050::begin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    // 400kHz (fast mode, suportado pelo MPU6050) em vez dos 100kHz padrão.
    // Necessário para o Modo Vibração em taxa alta: cada amostra faz duas
    // transações de 14 bytes (uma por eixo), que a 100kHz custam ~2,8ms
    // somadas — mais que o período de 2ms de uma captura a 500 amostras/s,
    // ou seja, o barramento simplesmente não daria conta. A 400kHz o mesmo
    // par custa ~0,7ms, com folga confortável.
    Wire.setClock(I2C_CLOCK_HZ);
    return configure();
}

bool Mpu6050::configure() {
    // Liga o filtro passa-baixa interno e fixa o fundo de escala — ver a
    // justificativa de cada valor em Mpu6050.h. A banda é a corrente (a do
    // Modo Vibração, se houver captura em andamento).
    //
    // Tudo isso ANTES de acordar o chip (os registradores aceitam escrita em
    // sleep): depois de um reset ele volta em +-250°/s, e acordado com uma
    // escrita do meio falhando ele mediria o giro 4x maior do que o driver
    // converte até a próxima checagem — 1 s de pan integrado errado. Dormindo,
    // lê zeros, que readMotion() já descarta.
    if (!writeRegister(REG_CONFIG, _dlpfCfg)) {
        return false;
    }
    if (!writeRegister(REG_ACCEL_CONFIG, ACCEL_RANGE_2G)) {
        return false;
    }
    if (!writeRegister(REG_GYRO_CONFIG, GYRO_RANGE_1000DPS)) {
        return false;
    }
    // Sai do modo sleep (padrão de fábrica) e usa o clock interno.
    return writeRegister(REG_PWR_MGMT_1, 0x00);
}

void Mpu6050::maintain() {
    uint32_t now = millis();
    bool busStuck = _consecutiveFailures >= MPU_BUS_RESET_FAILURES;
    if (!busStuck && now - _lastHealthCheckMs < MPU_HEALTH_CHECK_INTERVAL_MS) {
        return;
    }
    _lastHealthCheckMs = now;

    uint8_t pwr = 0, cfg = 0, gyro = 0, accel = 0;
    bool readOk = readRegister(REG_PWR_MGMT_1, pwr) && readRegister(REG_CONFIG, cfg) &&
                  readRegister(REG_GYRO_CONFIG, gyro) && readRegister(REG_ACCEL_CONFIG, accel);
    bool configOk = readOk && pwr == 0x00 && (cfg & 0x07) == _dlpfCfg &&
                    (gyro & 0x18) == GYRO_RANGE_1000DPS && (accel & 0x18) == ACCEL_RANGE_2G;
    if (configOk && !busStuck) {
        return;
    }
    if (!readOk || busStuck) {
        // Barramento sem resposta: reinicia o periférico I2C do ESP32 antes
        // de reconfigurar (um escravo segurando SDA baixo só solta assim).
        Wire.end();
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
        Wire.setClock(I2C_CLOCK_HZ);
    }
    configure();
    _recoveries++;
    _consecutiveFailures = 0;
}

bool Mpu6050::readRegister(uint8_t reg, uint8_t &value) {
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom(I2C_ADDRESS, static_cast<uint8_t>(1)) != 1) {
        return false;
    }
    value = static_cast<uint8_t>(Wire.read());
    return true;
}

bool Mpu6050::accelPlausible(int16_t rawX, int16_t rawY, int16_t rawZ) {
    float x = rawX / ACCEL_SENSITIVITY_LSB_PER_G;
    float y = rawY / ACCEL_SENSITIVITY_LSB_PER_G;
    float z = rawZ / ACCEL_SENSITIVITY_LSB_PER_G;
    float norm2 = x * x + y * y + z * z;
    return norm2 >= ACCEL_MIN_PLAUSIBLE_G * ACCEL_MIN_PLAUSIBLE_G &&
           norm2 <= ACCEL_MAX_PLAUSIBLE_G * ACCEL_MAX_PLAUSIBLE_G;
}

bool Mpu6050::fail() {
    _readFailures++;
    if (_consecutiveFailures < UINT16_MAX) {
        _consecutiveFailures++;
    }
    return false;
}

bool Mpu6050::setDlpfForSampleRate(uint16_t rateHz) {
    // Banda do filtro escolhida bem abaixo de Nyquist (rateHz/2), para ele
    // continuar servindo de anti-aliasing, mas alta o bastante para não
    // apagar o que a taxa maior foi feita para medir. As bandas disponíveis
    // no chip são discretas (21/44/94/184/260Hz).
    uint8_t cfg = DLPF_CFG_21HZ;
    if (rateHz >= 400) {
        cfg = DLPF_CFG_94HZ;  // Nyquist >= 200Hz: 2,1x de margem
    } else if (rateHz >= 200) {
        cfg = DLPF_CFG_44HZ;  // Nyquist >= 100Hz: 2,3x de margem
    }
    _dlpfCfg = cfg;  // maintain() compara o chip com isto
    return writeRegister(REG_CONFIG, cfg);
}

bool Mpu6050::restoreDefaultDlpf() {
    _dlpfCfg = DLPF_CFG_21HZ;
    return writeRegister(REG_CONFIG, DLPF_CFG_21HZ);
}

bool Mpu6050::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool Mpu6050::readAccelG(float &ax, float &ay, float &az) {
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) {
        return fail();
    }
    if (Wire.requestFrom(I2C_ADDRESS, static_cast<uint8_t>(6)) != 6) {
        return fail();
    }

    uint8_t buffer[6];
    for (uint8_t i = 0; i < 6; i++) {
        buffer[i] = static_cast<uint8_t>(Wire.read());
    }
    int16_t rawX = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
    int16_t rawY = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
    int16_t rawZ = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);
    if (!accelPlausible(rawX, rawY, rawZ)) {
        return fail();
    }
    _consecutiveFailures = 0;

    ax = rawX / ACCEL_SENSITIVITY_LSB_PER_G;
    ay = rawY / ACCEL_SENSITIVITY_LSB_PER_G;
    az = rawZ / ACCEL_SENSITIVITY_LSB_PER_G;
    return true;
}

bool Mpu6050::readMotion(float &ax, float &ay, float &az, float &gxDps, float &gyDps, float &gzDps) {
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) {
        return fail();
    }
    if (Wire.requestFrom(I2C_ADDRESS, static_cast<uint8_t>(MOTION_BURST_LEN)) != MOTION_BURST_LEN) {
        return fail();
    }

    // Bytes vão para um buffer antes de serem combinados, de propósito: em
    // `(Wire.read() << 8) | Wire.read()` a ordem de avaliação dos dois lados
    // do `|` não é garantida pelo padrão C++, então os bytes poderiam sair
    // trocados dependendo do compilador.
    uint8_t buffer[MOTION_BURST_LEN];
    for (uint8_t i = 0; i < MOTION_BURST_LEN; i++) {
        buffer[i] = static_cast<uint8_t>(Wire.read());
    }

    int16_t rawAx = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
    int16_t rawAy = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
    int16_t rawAz = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);
    // buffer[6..7] = temperatura, não usada (ver MOTION_BURST_LEN em Mpu6050.h)
    int16_t rawGx = static_cast<int16_t>((buffer[8] << 8) | buffer[9]);
    int16_t rawGy = static_cast<int16_t>((buffer[10] << 8) | buffer[11]);
    int16_t rawGz = static_cast<int16_t>((buffer[12] << 8) | buffer[13]);
    // Quadro implausível = lixo do barramento ou chip reiniciado (ver
    // ACCEL_MIN_PLAUSIBLE_G), e giro no fundo de escala = valor cortado,
    // que não serve para integrar. Descartados como falha de leitura.
    if (!accelPlausible(rawAx, rawAy, rawAz) || rawGx == INT16_MIN || rawGx == INT16_MAX ||
        rawGy == INT16_MIN || rawGy == INT16_MAX || rawGz == INT16_MIN || rawGz == INT16_MAX) {
        return fail();
    }
    _consecutiveFailures = 0;

    ax = rawAx / ACCEL_SENSITIVITY_LSB_PER_G;
    ay = rawAy / ACCEL_SENSITIVITY_LSB_PER_G;
    az = rawAz / ACCEL_SENSITIVITY_LSB_PER_G;
    gxDps = rawGx / GYRO_SENSITIVITY_LSB_PER_DPS;
    gyDps = rawGy / GYRO_SENSITIVITY_LSB_PER_DPS;
    gzDps = rawGz / GYRO_SENSITIVITY_LSB_PER_DPS;
    return true;
}
