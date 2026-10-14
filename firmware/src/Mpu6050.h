#pragma once

#include <stdint.h>

// Driver mínimo para o MPU6050 via I2C (sem biblioteca externa).
// Registradores e escalas default (acelerômetro +-2g, 16384 LSB/g;
// giroscópio +-1000°/s, 32,8 LSB/°/s — ver GYRO_RANGE_1000DPS) conforme o
// datasheet do MPU6050.
//
// Dois caminhos de leitura, para dois consumidores diferentes:
// - readAccelG(): só o acelerômetro (6 bytes), usado pelo AngleSensor para
//   o tilt e pela captura de vibração — é o caminho mais curto no
//   barramento, chamado nas taxas mais altas do firmware;
// - readMotion(): acelerômetro + giroscópio num burst único (14 bytes),
//   usado pelo PanSensor. O burst único importa: a compensação de tilt da
//   taxa de pan combina accel e giro, e os dois precisam ser da MESMA
//   amostra — duas transações separadas dariam instantes diferentes.
class Mpu6050 {
public:
    bool begin();

    // Preenche ax, ay, az em unidades de g. Retorna false em falha de I2C.
    bool readAccelG(float &ax, float &ay, float &az);

    // Preenche acelerômetro (g) e giroscópio (graus/s) da mesma amostra,
    // num único burst I2C. Retorna false em falha de I2C.
    bool readMotion(float &ax, float &ay, float &az, float &gxDps, float &gyDps, float &gzDps);

    // Ajusta a banda do filtro interno (DLPF) à taxa de amostragem pedida,
    // mantendo-a bem abaixo de Nyquist para servir de anti-aliasing sem
    // cortar o que se quer medir. Chamado pelo Modo Vibração no início da
    // captura; `restoreDefaultDlpf()` devolve a banda estreita (21Hz) usada
    // pela leitura contínua, que precisa de estabilidade, não de banda.
    bool setDlpfForSampleRate(uint16_t rateHz);
    bool restoreDefaultDlpf();

    // Chamar a cada iteração do loop(). A cada MPU_HEALTH_CHECK_INTERVAL_MS
    // relê a configuração do chip e, se ela não bater com a esperada (sensor
    // reiniciado por queda de alimentação volta em SLEEP, entregando zeros),
    // reconfigura. Com MPU_BUS_RESET_FAILURES leituras seguidas falhando,
    // reinicia também o periférico I2C, sem esperar o intervalo.
    void maintain();

    // Diagnóstico: reconfigurações feitas por maintain() e leituras
    // descartadas (falha de I2C ou quadro implausível) desde o boot.
    uint32_t recoveries() const { return _recoveries; }
    uint32_t readFailures() const { return _readFailures; }

private:
    static constexpr uint8_t I2C_ADDRESS = 0x68;
    static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
    static constexpr uint8_t REG_CONFIG = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    static constexpr float ACCEL_SENSITIVITY_LSB_PER_G = 16384.0f;
    static constexpr float GYRO_SENSITIVITY_LSB_PER_DPS = 32.8f;

    // Bloco contíguo a partir de REG_ACCEL_XOUT_H: 6 bytes de acelerômetro,
    // 2 de temperatura e 6 de giroscópio. A temperatura fica no meio e não
    // pode ser pulada — por isso o burst é de 14 bytes e não de 12.
    static constexpr uint8_t MOTION_BURST_LEN = 14;

    // Filtro passa-baixa digital interno do MPU6050 (registrador CONFIG,
    // campo DLPF_CFG). O padrão de fábrica é 0 = 260Hz de banda passante no
    // acelerômetro, ou seja, praticamente nenhum filtro: todo o ruído
    // mecânico/térmico de alta frequência entra em cada amostra e vira
    // tremulação de centésimos de grau na leitura de ângulo.
    //
    // DLPF_CFG=4 -> 21Hz de banda (atraso de ~8.5ms), que:
    // - corta a maior parte desse ruído já no hardware, antes de qualquer
    //   filtragem em software;
    // - serve de filtro anti-aliasing para o Modo Vibração: na taxa padrão
    //   de 50 amostras/s, Nyquist é 25Hz — com os 260Hz de fábrica, ruído
    //   acima disso era rebatido para dentro da faixa medida e sujava a FFT.
    // Ainda deixa passar com folga as frequências de interesse do ensaio
    // (balanço de mastro sob vento, tipicamente 1-5Hz).
    static constexpr uint8_t DLPF_CFG_21HZ = 0x04;

    // Demais bandas do mesmo filtro, usadas pelo Modo Vibração em taxas de
    // amostragem altas (ver setDlpfForSampleRate). Em 500 amostras/s,
    // Nyquist é 250Hz e manter a banda em 21Hz jogaria fora justamente o
    // que a taxa alta foi feita para enxergar: o sinal sairia do chip já
    // filtrado a 21Hz e a taxa maior não mediria nada de novo.
    static constexpr uint8_t DLPF_CFG_44HZ = 0x03;
    static constexpr uint8_t DLPF_CFG_94HZ = 0x02;

    // Fundo de escala do acelerômetro: +-2g (o mais sensível, ideal para
    // inclinação). É o padrão de fábrica, mas fica explícito para o driver
    // não depender do estado em que o chip foi encontrado.
    static constexpr uint8_t ACCEL_RANGE_2G = 0x00;

    // Fundo de escala do giroscópio: +-1000°/s (32,8 LSB/°/s). Até a 1.6.14
    // era +-250°/s, pensado para o motor (~20-30°/s) — mas o pan também é
    // girado À MÃO, e um giro de 90° em meio segundo passa de 350°/s no pico.
    // Amostra no fundo de escala é descartada (valor cortado), e o trecho
    // saturado era integrado com a taxa de antes/depois dele: +90° rápido
    // chegou a ler 79-82° em simulação, com erro acumulando giro a giro. A
    // resolução de 0,03°/s por LSB continua bem abaixo do ruído do sensor
    // (~0,05°/s por amostra) e do limiar de parado do ZUPT.
    static constexpr uint8_t GYRO_RANGE_1000DPS = 0x10;

    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t &value);
    bool configure();

    // Aceleração com módulo fora de [ACCEL_MIN_PLAUSIBLE_G,
    // ACCEL_MAX_PLAUSIBLE_G] não é física para o equipamento montado: é
    // lixo do barramento (bytes 0xFF), chip reiniciado (zeros) ou choque.
    static bool accelPlausible(int16_t rawX, int16_t rawY, int16_t rawZ);
    bool fail();  // conta a falha e devolve false

    uint8_t _dlpfCfg = DLPF_CFG_21HZ;  // banda que o chip DEVE estar usando
    uint32_t _lastHealthCheckMs = 0;
    uint16_t _consecutiveFailures = 0;
    uint32_t _recoveries = 0;
    uint32_t _readFailures = 0;
};
