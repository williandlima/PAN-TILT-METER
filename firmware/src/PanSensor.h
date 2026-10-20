#pragma once

#include <stdint.h>

#include "Config.h"
#include "Mpu6050.h"
#include "PeakHold.h"

// Calcula o ângulo de azimute (pan) — a rotação em torno da vertical —
// integrando o GIROSCÓPIO do MPU6050, relativo a um zero calibrado.
//
// Por que não dá pelo acelerômetro: ele mede a direção do vetor gravidade, e
// girar em torno da vertical não muda esse vetor. É limitação física, não de
// código. O giroscópio, por outro lado, mede velocidade angular em torno de
// qualquer eixo — inclusive o vertical.
//
// São quatro mecanismos trabalhando juntos:
//
// 1. COMPENSAÇÃO DE TILT. O sensor está montado na parte que inclina, então o
//    eixo Z dele não aponta para a vertical: ele está inclinado do próprio
//    ângulo de tilt. Uma rotação de pan (velocidade angular ao longo da
//    vertical) se projeta nos eixos do corpo, e pegar gz cru subestimaria a
//    taxa por cos(tilt) — a 60° de tilt a leitura sairia pela metade. A taxa
//    correta é a projeção do vetor de velocidade angular medido sobre a
//    vertical escrita em coordenadas do corpo:
//
//        ω_pan = gz*cos(θ) + gy*sin(θ),   θ = atan2(ay, az)
//
//    O acelerômetro parado mede a vertical (para cima) nos eixos do chip,
//    (0, sin θ, cos θ), e o giro mede a velocidade angular nos MESMOS eixos;
//    girando em torno da vertical a Ω, gy = Ω·sin θ e gz = Ω·cos θ. A fórmula
//    é o produto escalar dos dois e não depende de como a placa foi montada.
//    Até a 1.6.14 o termo de gy tinha o sinal trocado: a taxa saía
//    multiplicada por cos(2θ) — 6% a menos a 10° de tilt, metade a 30°, zero
//    a 45° e com o SENTIDO invertido acima disso. Os testes não pegavam
//    porque o sensor simulado tinha o mesmo sinal trocado.
//
//    A projeção dá positivo no sentido ANTI-HORÁRIO visto de cima (regra da
//    mão direita em torno da vertical para cima), qualquer que seja a
//    montagem. O pan do usuário é positivo no sentido HORÁRIO visto de cima
//    (PAN_DIRECTION_SIGN = -1, aplicado em projectedDps()).
//
//    Sendo uma projeção (produto escalar com um vetor unitário), e não a
//    fórmula de taxa de Euler (gy*sinφ + gz*cosφ)/cosθ, não há singularidade
//    em nenhum ângulo de tilt. E ela rejeita sozinha a taxa do próprio tilt,
//    que acontece em torno do eixo X e portanto não tem componente na
//    vertical. O accel e o giro vêm do mesmo burst I2C (Mpu6050::readMotion),
//    então θ e as taxas são do mesmo instante.
//
// 2. ZUPT (zero-rate update). Integrar giro acumula erro: um bias residual
//    vira uma rampa. Mas o drift só corre enquanto se integra — e o eixo de
//    pan aqui fica parado a maior parte do tempo. A cada
//    PAN_ZUPT_WINDOW_MS calcula-se a média de gy e gz; se ela estiver
//    próxima do bias corrente, a janela é considerada parada e o bias é
//    atualizado em direção a essa média. O gate usa a MÉDIA (e não a
//    dispersão) de propósito: vibração é de média zero, então o mastro pode
//    estar balançando sob vento que a janela ainda é reconhecida como parada
//    — o limiar só cresce com a dispersão da janela na medida do resíduo
//    que a vibração deixa na média (PAN_ZUPT_VIBRATION_FACTOR).
//
//    Janela que NÃO passa pelo gate é movimento e não ensina nada ao bias.
//    Até a 1.6.13 o limiar era 1°/s, e um giro à mão lento passava inteiro
//    por "parado": era apagado e ainda empurrava o bias na direção da própria
//    velocidade — depois de parar, o bias errado puxava a leitura para o lado
//    contrário (-0,6° depois de um giro positivo) e ela levava ~1 min para
//    voltar. Com o limiar em 0,1°/s, o que sobra são as BORDAS do giro: a
//    partida e a frenagem suaves de um giro à mão passam algumas janelas
//    abaixo dele. Elas são reconhecidas pela forma — mesmo sentido do giro e
//    velocidade crescendo (partida) ou caindo (frenagem) janela a janela, o
//    que a deriva térmica, praticamente constante entre janelas, não faz:
//    na partida, quando o giro é detectado, o que foi cancelado nelas volta e
//    o que ensinaram ao bias é desfeito; na frenagem, continuam sendo giro.
//    Ver PAN_ZUPT_EDGE_* em Config.h e os cenários K-N de sim/pan_sim.cpp.
//
//    O outro lado de um limiar baixo: o zero do giro anda com o eixo parado
//    (aquece por minutos depois de ligar; passeia). Andando um pouco mais que
//    o limiar entre duas janelas paradas, todas as seguintes saíam dele e,
//    como janela em movimento não ensina o bias, ele nunca mais voltava — a
//    diferença era integrada sem fim (1.6.14, em bancada: -30/+90/+30/+60/0
//    leu -35,6/+83,5/+20,2/+49,1/-7,5). Duas defesas (PAN_ZUPT_SHIFT_* e
//    PAN_ZUPT_WANDER_* em Config.h): janelas seguidas sem giro de verdade e
//    com médias sobre uma reta lisa, ou longas o bastante, são o eixo parado
//    sob um bias que andou — o bias é readotado e o que elas integraram é
//    desfeito; e o limiar sobe com o passeio medido do próprio giro.
//    Cenários P-Q de sim/pan_sim.cpp. Desde a 1.6.16, as janelas lentas
//    coladas a um giro e no sentido dele (a aproximação final da marca) não
//    são apagadas nessa hora — só a parte do bias errado sai delas
//    (cenário T).
//
//    O bias é estimado POR EIXO, no referencial do sensor (gy, gz), e só
//    depois projetado na vertical — nunca sobre ω_pan já projetado. O bias
//    de fábrica é fixo nos eixos do chip; projetado, ele vale
//    bgz·cos(θ) + bgy·sin(θ) e portanto MUDA com o tilt. Estimado sobre
//    ω_pan, qualquer mudança de inclinação deixava o bias aprendido errado
//    por alguns °/s, o gate passava a rejeitar toda janela parada (o bias
//    nunca mais se corrigia) e esse erro era integrado sem fim até cravar a
//    leitura em ±PAN_MAX_DEG — medido em simulação: 45° de tilt com bias de
//    fábrica típico dava ~1,4°/s de deriva, travando em -90° em ~60 s.
//    A distância usada no gate, hypot(Δgy, Δgz), vale exatamente |ω_pan|
//    para uma rotação de pan em qualquer tilt, então o limiar mantém o
//    significado de "°/s de pan". Rotação de tilt (eixo X) não aparece em
//    gy/gz e não tira a janela da condição de parada.
//
// 3. CANCELAMENTO DE JANELA PARADA. Não basta parar de corrigir o bias — se a
//    integração continuasse rodando enquanto parado, o ruído do giro viraria
//    random walk no ângulo. Então o quanto foi integrado dentro da janela é
//    acumulado à parte e, se a janela fechar classificada como parada, é
//    subtraído de volta. Parado, o ângulo fica cravado. O preço é o piso de
//    detecção: um movimento isolado cujo deslocamento total fique abaixo de
//    PAN_ZUPT_RATE_THRESHOLD_DPS * janela (~0,1°) é descartado como ruído.
//
// 4. FATOR DE ESCALA (PAN_SCALE_CORRECTION). Resolvido o bias, o erro
//    dominante passa a ser a tolerância de sensibilidade do giro (~±3% de
//    fábrica). Ele é proporcional ao deslocamento ATUAL em relação ao zero —
//    não se acumula com o tempo nem com o número de movimentos — e some
//    quando o eixo volta ao zero.
//
// LEITURAS ESPÚRIAS. gy e gz passam por uma mediana móvel de 5 amostras
// antes de tudo (integração e médias do ZUPT). Uma única leitura errada do
// barramento I2C — ex.: -250°/s por 10 ms — deslocava a média da janela em
// 2,5°/s: a janela deixava de ser "parada", e os ~2,5° que ela integrou
// ficavam para sempre. Algumas por minuto, com o sinal puxando para o mesmo
// lado, levavam a placa PARADA a cravar em ±90° em poucos minutos (até a
// 1.6.3; cenário I de sim/pan_sim.cpp). A mediana descarta até 2 amostras
// ruins seguidas e custa ~20 ms de atraso, irrelevante perto do motor.
// Movimento real passa intacto: o DLPF de 21 Hz já não deixa o sinal
// verdadeiro mudar de forma brusca entre amostras de 10 ms. O Modo
// Vibração (readInstantRateDps) não passa pela mediana. Antes dela, uma
// amostra que salta mais que PAN_GYRO_SLEW_MAX_DPS da anterior é descartada
// como leitura falha: com o fundo de escala de ±1000°/s, um quadro de lixo
// traz giro de até 1000°/s, e três seguidos venciam a mediana. Efeito
// colateral da mediana: perto de submúltiplos da amostragem (33 Hz = 100/3, 14,3 Hz = 100/7) a
// mediana RETIFICA vibração do mastro num bias falso que muda devagar — por
// isso o limiar de "parado" cresce com a vibração medida nas amostras
// brutas (PAN_ZUPT_VIBRATION_FACTOR).
//
// BOOT E RECUPERAÇÃO. O bias inicial não pode passar pelo gate do item 2,
// porque o zero-rate de fábrica do MPU6050 chega a ±20°/s. Ele é adotado
// quando DUAS janelas seguidas têm médias coerentes entre si (distância
// abaixo de PAN_BIAS_ADOPT_COHERENCE_DPS), como a média das duas: placa
// manuseada ao ligar tem média variando de janela a janela e é recusada;
// vibração, de média zero, não atrapalha. Até lá o ângulo fica congelado
// (~2 s após o boot/calibração).
//
// Antes (≤1.6.1) a PRIMEIRA janela era aceita sem teste. Ligada em
// movimento (típico no uso via BLE, com a placa na mão; via USB o app
// reinicia o ESP32 ao abrir a porta, com a placa parada na bancada), o bias
// saía errado, o gate recusava toda janela parada dali em diante e a
// leitura derivava até cravar em ±PAN_MAX_DEG para sempre.
//
// Rede de segurança para o caso raro de o bias ainda sair errado (ex.: giro
// lento e constante bem na hora do boot): PAN_BIAS_RELEARN_WINDOWS janelas
// seguidas fora do gate, com médias sobre uma reta (bias constante ou
// derivando), só podem ser o eixo parado sob um bias errado — nenhuma
// varredura do motor dura tanto dentro do curso, e um giro à mão não mantém
// a velocidade tão lisa (PAN_BIAS_RELEARN_RESIDUAL_DPS). Nesse caso o bias é
// reaprendido (nível e deriva da reta) e o que foi integrado nessas janelas
// é desfeito. Até a 1.6.13 bastava janelas vizinhas "coerentes" a 1°/s — e
// todo giro à mão lento e suave passava nesse teste.
class PanSensor {
public:
    explicit PanSensor(Mpu6050 &mpu) : _mpu(mpu) {}

    // Chamar a cada iteração do loop(): amostra o sensor em intervalo fixo
    // (ANGLE_SAMPLE_INTERVAL_MS, o mesmo do tilt), integra e mantém o ZUPT.
    void update();

    // Ângulo de pan relativo à calibração (graus), já clampado a
    // [PAN_MIN_DEG, PAN_MAX_DEG]. Não faz I2C: devolve o estado do
    // integrador mantido por update().
    float readPanDeg();

    // Extremos do pan acumulados desde o último reset/calibração, medidos a
    // 100 Hz (ver PeakHold.h). Aqui o peak-hold é SIMPLES, sem filtro extra e
    // sem teste de persistência, ao contrário do tilt — e por dois motivos:
    //
    // - não há ruído a filtrar. O ângulo de pan não vem do acelerômetro, vem
    //   de uma integração cujo mecanismo de cancelamento de janela parada
    //   (item 3 do cabeçalho acima) já deixa o valor cravado com o eixo
    //   parado. Filtrar de novo só adicionaria atraso;
    // - persistência aqui custaria caro. O motor gira a ~20-30°/s, então
    //   100 ms de exigência de persistência cortariam 2-3° do ponto de
    //   retorno de cada varredura — justamente o extremo que se quer registrar.
    //
    // O que o peak-hold resolve no pan é o outro problema: a 4-5 Hz de
    // polling, uma varredura rápida do motor passa pelo extremo entre duas
    // leituras do app e some.
    //
    // Efeito de borda conhecido: um movimento pequeno o bastante para a janela
    // ainda ser classificada como parada (< ~0,1°, ver item 3) é cancelado no
    // ângulo mas fica registrado no extremo, porque o peak-hold já o viu
    // acontecer. O extremo é o valor honesto dos dois — o movimento existiu.
    bool hasPeaks() const { return _peaks.hasData(); }
    float minPanDeg();
    float maxPanDeg();

    // Esquece os extremos sem mexer no zero — o botão de reset dos apps.
    void resetPeaks() { _peaks.reset(); }

    // Zera o pan na posição atual e reinicia a estimativa de bias do giro —
    // a próxima janela vira a nova referência de "parado". Chamado junto com
    // AngleSensor::calibrate(), pela mesma ação de calibração dos apps.
    void calibrate();

    // true depois que duas janelas coerentes estabeleceram o bias (até lá o
    // ângulo fica congelado, ~2 s após o boot).
    bool biasReady() const { return _biasReady; }

    // Velocidade angular instantânea do eixo de pan, em graus/s, já
    // compensada por tilt, com o bias estimado subtraído e o fator de escala
    // aplicado. Faz uma leitura I2C na hora.
    //
    // É ESTA a grandeza usada pelo Modo Vibração do pan — e não o ângulo
    // integrado de readPanDeg(). O motivo é o mecanismo 3 do cabeçalho
    // acima: um ensaio de vibração acontece justamente com o eixo parado,
    // então o cancelamento de janela parada apagaria de propósito toda a
    // oscilação que se quer medir. A taxa não passa por integração nem por
    // cancelamento, e ainda deixa o bias residual concentrado em 0 Hz, onde
    // a análise espectral já o descarta por construção.
    float readInstantRateDps();

    // Estado interno para diagnóstico em campo (characteristic
    // CHAR_PAN_DIAGNOSTICS_UUID, lida por python-app/tools/diagnostico_pan.py).
    struct Diagnostics {
        float panDeg;       // integrador interno, sem offset e sem clamp
        float offsetDeg;
        float biasGyDps;
        float biasGzDps;
        float meanGyDps;    // média da última janela fechada
        float meanGzDps;
        float gyDps;        // última amostra crua
        float gzDps;
        float tiltDeg;      // atan2(ay, az) da última amostra
        uint32_t samples;   // amostras válidas desde o boot
        uint32_t i2cFailures;
        uint16_t mismatchWindows;
        uint8_t biasReady;
        uint32_t spikes;    // leituras espúrias descartadas pela mediana
        uint32_t mpuRecoveries;  // reconfigurações do sensor (Mpu6050::maintain)
    };
    Diagnostics diagnostics() const;

private:
    Mpu6050 &_mpu;

    float _panDeg = 0.0f;     // integrado desde o boot, limitado à faixa (clampIntegrator)
    float _offsetDeg = 0.0f;  // zero da calibração
    // Bias estimado de cada eixo do giro, no referencial do sensor (°/s) —
    // ver item 2 do cabeçalho: não pode ser estimado sobre ω_pan projetado.
    float _biasGyDps = 0.0f;
    float _biasGzDps = 0.0f;
    // Velocidade de deriva do bias (°/s por s), estimada nas janelas paradas.
    float _biasSlopeGy = 0.0f;
    float _biasSlopeGz = 0.0f;
    float _sinceStillS = 0.0f;  // tempo desde a última janela parada
    static float clampSlope(float slope);
    bool _biasReady = false;  // primeira janela já definiu o bias?
    // Bias já confirmado por uma janela parada (ou adotado com a placa
    // parada de fato). Enquanto não, a rede de segurança é mais permissiva —
    // ver PAN_BIAS_RELEARN_RESIDUAL_PROVISIONAL_DPS.
    bool _biasConfirmed = false;

    uint32_t _lastSampleMs = 0;   // última leitura BEM-SUCEDIDA
    uint32_t _lastAttemptMs = 0;
    bool _failedAttempt = false;
    bool _hasLastSample = false;  // primeira amostra não tem dt confiável
    float _lastRateDps = 0.0f;    // última taxa válida (repetida em falha de I2C)
    float _prevIntegRateDps = 0.0f;  // taxa da amostra anterior (trapézio)
    bool _hasPrevIntegRate = false;

    // Estado da janela de ZUPT em andamento.
    uint32_t _windowStartMs = 0;
    float _windowGySumDps = 0.0f;
    float _windowGzSumDps = 0.0f;
    uint16_t _windowSamples = 0;
    float _windowDeltaDeg = 0.0f;  // quanto foi integrado nesta janela
    float _windowPeakDps = 0.0f;   // maior |taxa - bias| da janela (PAN_ZUPT_PEAK_DPS)
    // Quadrado da diferença entre amostras brutas sucessivas, para medir
    // vibração na janela (limiar adaptativo, PAN_ZUPT_VIBRATION_FACTOR).
    float _windowDiffSqSum = 0.0f;
    uint16_t _windowDiffCount = 0;
    float _prevRawGyDps = 0.0f;
    float _prevRawGzDps = 0.0f;
    bool _hasPrevRaw = false;

    // Estado do bias antes de cada uma das últimas janelas paradas, para
    // desfazer o que o começo suave de um giro ensinou a ele (ver
    // PAN_ZUPT_BIAS_UNDO_WINDOWS).
    struct BiasSnapshot {
        float gy, gz, slopeGy, slopeGz;  // bias usado na janela (no fim dela)
        uint32_t ms;                     // fim da janela
        float sinceStillS;
        float deltaDeg;   // integrado na janela (e cancelado)
        float signedDps;  // desvio da média, projetado na vertical
        float spreadDps;  // dispersão das amostras
        float winS;
        float tiltRad;
        float meanGy, meanGz;  // médias cruas da janela
    };
    static constexpr int UNDO_LEN = PAN_ZUPT_BIAS_UNDO_WINDOWS;
    BiasSnapshot _undo[UNDO_LEN] = {};
    int _undoCount = 0;
    void pushBiasUndo(uint32_t now, float winS, float signedDps, float spreadDps, float meanGy, float meanGz);
    void undoRecentBiasLearning(uint32_t now, float winS, float motionDps);
    // Componente vertical (taxa de pan) de uma diferença de taxa nos eixos.
    static float projectedDps(float dGy, float dGz, float tiltRad);
    // Janela de borda (começo/fim suave) do giro de velocidade motionDps.
    static bool isEdgeOf(float edgeDps, float spreadDps, float motionDps);

    // Estado do último giro, para reconhecer a cauda de uma frenagem suave.
    bool _prevMoving = false;
    uint16_t _tailWindows = 0;
    float _lastMovingDps = 0.0f;

    // Janelas da excursão corrente (desde a última janela parada), para
    // reconhecer um bias que mudou com o eixo parado (PAN_ZUPT_SHIFT_*).
    struct ExcursionWindow {
        float meanGy, meanGz;  // médias cruas
        float deltaDeg;        // integrado (e mantido) na janela
        float winS, tiltRad, spreadDps;
        float devDps;          // |média - bias|
        float biasGy, biasGz;  // bias com que a janela foi integrada (no meio dela)
        uint32_t ms;           // fim da janela
        bool quiet;            // sem giro de verdade (abaixo de SHIFT_MAX)
    };
    static constexpr int EXC_LEN = PAN_ZUPT_SHIFT_HISTORY;
    ExcursionWindow _exc[EXC_LEN] = {};
    int _excCount = 0;
    // Passo quadrático médio da média entre janelas paradas seguidas
    // (PAN_ZUPT_WANDER_*), e se a janela anterior foi parada.
    float _wanderVar = 0.0f;
    bool _prevStill = false;
    void learnWander(float stepGy, float stepGz);
    void pushExcursion(const ExcursionWindow &w);
    // Adota o bias que mudou e desfaz a deriva integrada; true se adotou.
    bool tryShiftEscape();

    // Média da janela anterior, para o teste de coerência (BOOT E RECUPERAÇÃO).
    bool _hasPrevMean = false;
    float _prevMeanGyDps = 0.0f;
    float _prevMeanGzDps = 0.0f;

    // Últimas janelas SEGUIDAS fora do limiar de "parado" (rede de segurança
    // contra bias errado): média de cada eixo e o quanto cada uma integrou.
    static constexpr int RELEARN_LEN = PAN_BIAS_RELEARN_WINDOWS;
    float _relearnGy[RELEARN_LEN] = {};
    float _relearnGz[RELEARN_LEN] = {};
    float _relearnDeltaDeg[RELEARN_LEN] = {};
    float _relearnSpreadDps[RELEARN_LEN] = {};
    uint16_t _relearnCount = 0;  // janelas guardadas (<= RELEARN_LEN)
    uint16_t _relearnHead = 0;   // próxima posição de escrita
    void pushRelearnWindow(float meanGyDps, float meanGzDps, float deltaDeg, float spreadDps);
    // Reaprende o bias se as janelas guardadas caem sobre uma reta.
    void tryRelearn(float winS);

    // Mediana móvel de gy/gz (ver "LEITURAS ESPÚRIAS" no cabeçalho).
    static constexpr int PAN_DESPIKE_LEN = 5;
    float _gyHist[PAN_DESPIKE_LEN] = {};
    float _gzHist[PAN_DESPIKE_LEN] = {};
    float _tiltHist[PAN_DESPIKE_LEN] = {};
    int _histCount = 0;         // amostras válidas no histórico (<= 5)
    float _pendingDtS = 0.0f;   // intervalo ainda não integrado (após buraco)
    uint32_t _spikeCount = 0;
    uint8_t _slewRejects = 0;   // amostras seguidas descartadas por salto (PAN_GYRO_SLEW_MAX_DPS)
    // Mediana de gy, gz e tilt; false enquanto o histórico não estiver cheio.
    bool despike(float &gyDps, float &gzDps, float &tiltRad);

    float _lastGyDps = 0.0f;
    float _lastGzDps = 0.0f;
    float _lastTiltRad = 0.0f;
    uint32_t _sampleCount = 0;
    uint32_t _i2cFailures = 0;

    PeakHold _peaks{1};  // sem teste de persistência — ver minPanDeg() acima

    // Fecha a janela corrente: decide se estava parada, atualiza o bias e
    // cancela a integração da janela em caso afirmativo.
    void closeWindow(uint32_t now);

    void resetWindow(uint32_t now);

    void adoptBias(float gyDps, float gzDps, float slopeGy, float slopeGz, bool confirmed);
    void clampIntegrator();

    // Lê o sensor: gy/gz crus (°/s, sem bias subtraído) e o tilt do mesmo
    // burst (rad). false em falha de I2C.
    bool sampleMotion(float &gyDps, float &gzDps, float &tiltRad);

    // Taxa de pan (°/s): bias de cada eixo subtraído no referencial do
    // sensor e só então projetado na vertical (item 1 do cabeçalho).
    float panRateDps(float gyDps, float gzDps, float tiltRad) const;
};
