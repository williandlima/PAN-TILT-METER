#include "BleServer.h"

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <math.h>
#include <string.h>

#include "Config.h"

namespace {
BLECharacteristic *angleChar = nullptr;
BLECharacteristic *panChar = nullptr;
BLECharacteristic *vibrationStatusChar = nullptr;
BLECharacteristic *vibrationDataChar = nullptr;
BLECharacteristic *vibrationPanDataChar = nullptr;
BLECharacteristic *peaksChar = nullptr;
BLECharacteristic *panDiagnosticsChar = nullptr;
BleServer *self = nullptr;  // única instância — usada pelos callbacks estáticos do BLE

// Sinalizado pelo callback de desconexão e consumido no loop principal.
volatile bool restartAdvertisingPending = false;

// Orçamento de handles GATT do serviço. createService(const char*) reserva só
// 15, e cada characteristic consome 2 (declaração + valor) mais 1 por
// descritor BLE2902 — este serviço usa 31. Estourando o orçamento a lib não
// dá erro: só deixa de registrar, em silêncio, as characteristics que não
// couberam (era o que fazia o app não achar as do Modo Vibração nem a da
// versão do firmware). Refazer a conta ao adicionar characteristic.
constexpr uint32_t SERVICE_NUM_HANDLES = 40;

class ServerCallbacks : public BLEServerCallbacks {
    void onDisconnect(BLEServer *) override {
        // O ESP32 PARA de anunciar assim que um cliente conecta, e a lib BLE
        // do Arduino não volta a anunciar sozinha quando ele desconecta — o
        // dispositivo simplesmente some do scan até ser resetado na mão.
        // Aparece na hora de trocar de app (ou de reabrir o mesmo) e parece
        // hardware com defeito.
        //
        // Só marca aqui: quem realmente reinicia o anúncio é update(), no
        // loop principal. Este callback roda na task do stack BLE, e chamar
        // startAdvertising() de dentro dele, no meio do processamento da
        // desconexão, é justamente o caso em que essa chamada falha calada.
        restartAdvertisingPending = true;
    }
};

class CalibrateCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristic) override {
        std::string value = characteristic->getValue();
        if (!value.empty() && static_cast<uint8_t>(value[0]) == 0x01 && self != nullptr) {
            self->handleCalibrateWrite();
        }
    }
};

class ResetPeaksCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristic) override {
        std::string value = characteristic->getValue();
        if (!value.empty() && static_cast<uint8_t>(value[0]) == 0x01 && self != nullptr) {
            self->handleResetPeaksWrite();
        }
    }
};

class VibrationConfigCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristic) override {
        std::string value = characteristic->getValue();
        if (value.size() < 4 || self == nullptr) {
            return;
        }
        uint16_t durationS = static_cast<uint8_t>(value[0]) | (static_cast<uint16_t>(static_cast<uint8_t>(value[1])) << 8);
        uint16_t rateHz = static_cast<uint8_t>(value[2]) | (static_cast<uint16_t>(static_cast<uint8_t>(value[3])) << 8);
        self->handleVibrationConfigWrite(durationS, rateHz);
    }
};

class VibrationResendCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *characteristic) override {
        std::string value = characteristic->getValue();
        if (value.size() < 3 || self == nullptr) {
            return;
        }
        uint16_t startIndex = static_cast<uint8_t>(value[0]) | (static_cast<uint16_t>(static_cast<uint8_t>(value[1])) << 8);
        bool pan = static_cast<uint8_t>(value[2]) != 0;
        self->handleVibrationResendWrite(startIndex, pan);
    }
};
}  // namespace

void BleServer::begin() {
    self = this;

    BLEDevice::init(BLE_DEVICE_NAME);
    BLEDevice::setMTU(247);  // best-effort: reduz o nº de pacotes se o central negociar MTU maior
    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());
    BLEService *service = server->createService(BLEUUID(SERVICE_UUID), SERVICE_NUM_HANDLES);

    angleChar = service->createCharacteristic(
        CHAR_ANGLE_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    angleChar->addDescriptor(new BLE2902());

    panChar = service->createCharacteristic(
        CHAR_PAN_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    panChar->addDescriptor(new BLE2902());

    peaksChar = service->createCharacteristic(
        CHAR_PEAKS_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    peaksChar->addDescriptor(new BLE2902());

    BLECharacteristic *calibrateChar =
        service->createCharacteristic(CHAR_CALIBRATE_UUID, BLECharacteristic::PROPERTY_WRITE);
    calibrateChar->setCallbacks(new CalibrateCallbacks());

    BLECharacteristic *resetPeaksChar =
        service->createCharacteristic(CHAR_RESET_PEAKS_UUID, BLECharacteristic::PROPERTY_WRITE);
    resetPeaksChar->setCallbacks(new ResetPeaksCallbacks());

    BLECharacteristic *vibrationConfigChar =
        service->createCharacteristic(CHAR_VIBRATION_CONFIG_UUID, BLECharacteristic::PROPERTY_WRITE);
    vibrationConfigChar->setCallbacks(new VibrationConfigCallbacks());

    vibrationStatusChar =
        service->createCharacteristic(CHAR_VIBRATION_STATUS_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    vibrationStatusChar->addDescriptor(new BLE2902());

    vibrationDataChar =
        service->createCharacteristic(CHAR_VIBRATION_DATA_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    vibrationDataChar->addDescriptor(new BLE2902());

    vibrationPanDataChar =
        service->createCharacteristic(CHAR_VIBRATION_PAN_DATA_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    vibrationPanDataChar->addDescriptor(new BLE2902());

    BLECharacteristic *vibrationResendChar =
        service->createCharacteristic(CHAR_VIBRATION_RESEND_UUID, BLECharacteristic::PROPERTY_WRITE);
    vibrationResendChar->setCallbacks(new VibrationResendCallbacks());

    BLECharacteristic *firmwareVersionChar =
        service->createCharacteristic(CHAR_FIRMWARE_VERSION_UUID, BLECharacteristic::PROPERTY_READ);
    uint8_t versionPayload[2] = {
        static_cast<uint8_t>(FIRMWARE_VERSION_CODE & 0xFF),
        static_cast<uint8_t>(FIRMWARE_VERSION_CODE >> 8),
    };
    firmwareVersionChar->setValue(versionPayload, 2);

    panDiagnosticsChar =
        service->createCharacteristic(CHAR_PAN_DIAGNOSTICS_UUID, BLECharacteristic::PROPERTY_READ);

    service->start();

    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->start();
}

void BleServer::handleCalibrateWrite() {
    // Só agenda: quem calibra de fato é update(), no loop principal — este
    // callback roda na task do stack BLE, e calibrate() mexe nos mesmos
    // campos que panSensor.update()/angleSensor.update() escrevem a cada
    // iteração do loop (mesmo raciocínio de handleVibrationResendWrite).
    _calibratePending = true;
}

void BleServer::handleResetPeaksWrite() {
    _resetPeaksPending = true;
}

void BleServer::handleVibrationConfigWrite(uint16_t durationS, uint16_t rateHz) {
    // Só agenda: quem inicia de fato é o loop principal, via
    // VibrationCapture::update() — este callback roda na task do stack BLE
    // e start() mexe nos mesmos campos que update() usa.
    _vibration.requestStart(durationS, rateHz);
    _vibrationDataCursor = 0;
    _vibrationPanDataCursor = 0;
    _lastReportedVibrationStatus = VibrationCapture::Status::Capturing;
}

void BleServer::handleVibrationResendWrite(uint16_t startIndex, bool pan) {
    // Dois pedidos do mesmo eixo antes de o loop consumir: fica o menor
    // índice. A corrida possível aqui (o loop consumir entre o load e o
    // store) só causa uma retransmissão a mais, que é inofensiva.
    std::atomic<uint32_t> &request = pan ? _resendPanFrom : _resendTiltFrom;
    if (startIndex < request.load()) {
        request.store(startIndex);
    }
}

void BleServer::consumeResend(std::atomic<uint32_t> &request, uint16_t &cursor) {
    uint32_t from = request.exchange(NO_RESEND);
    if (from == NO_RESEND) {
        return;
    }
    // Rebobina o cursor do eixo pedido: a fase de envio recomeça dali e o app
    // recebe de novo tudo a partir daquele índice. Reenviar em excesso é
    // inofensivo — o app acumula as amostras por índice, então repetição é
    // idempotente —, e é bem mais simples (e mais robusto) do que enviar
    // exatamente um bloco isolado.
    if (from < cursor) {
        cursor = static_cast<uint16_t>(from);
    }
    // Também limpa o "pronto" já reportado: sem isso, o guard de
    // _lastReportedVibrationStatus impediria a nova notificação de conclusão
    // no fim da retransmissão, e o app ficaria esperando para sempre.
    if (_lastReportedVibrationStatus == VibrationCapture::Status::Ready) {
        _lastReportedVibrationStatus = VibrationCapture::Status::Capturing;
    }
}

void BleServer::notifyAngles() {
    uint32_t now = millis();
    if (now - _lastAngleNotifyMs < BLE_NOTIFY_INTERVAL_MS) {
        return;
    }
    _lastAngleNotifyMs = now;

    uint16_t rawAngle = static_cast<uint16_t>(lroundf(_sensor.readAngleDeg() * ANGLE_SCALE));
    uint8_t anglePayload[2] = {
        static_cast<uint8_t>(rawAngle & 0xFF),
        static_cast<uint8_t>(rawAngle >> 8),
    };
    angleChar->setValue(anglePayload, 2);
    angleChar->notify();

    uint16_t rawPan = static_cast<uint16_t>(lroundf(_pan.readPanDeg() * ANGLE_SCALE));
    uint8_t panPayload[2] = {
        static_cast<uint8_t>(rawPan & 0xFF),
        static_cast<uint8_t>(rawPan >> 8),
    };
    panChar->setValue(panPayload, 2);
    panChar->notify();

    notifyPeaks();
    updatePanDiagnostics();
}

void BleServer::updatePanDiagnostics() {
    PanSensor::Diagnostics d = _pan.diagnostics();
    const float floats[9] = {
        d.panDeg, d.offsetDeg, d.biasGyDps, d.biasGzDps, d.meanGyDps,
        d.meanGzDps, d.gyDps, d.gzDps, d.tiltDeg,
    };
    // 1.6.17+: eixo X e a vertical no fim (apps antigos leem só os 55
    // primeiros bytes). A vertical diz como o sensor está montado: o ângulo
    // do eixo X fora da horizontal é asin(up.x).
    const float tail[5] = {d.biasGxDps, d.gxDps, d.up.x, d.up.y, d.up.z};
    uint8_t payload[75];
    memcpy(payload, floats, sizeof(floats));  // ESP32 é little-endian
    memcpy(payload + 36, &d.samples, 4);
    memcpy(payload + 40, &d.i2cFailures, 4);
    memcpy(payload + 44, &d.mismatchWindows, 2);
    payload[46] = d.biasReady;
    memcpy(payload + 47, &d.spikes, 4);
    memcpy(payload + 51, &d.mpuRecoveries, 4);
    memcpy(payload + 55, tail, sizeof(tail));
    panDiagnosticsChar->setValue(payload, sizeof(payload));
}

void BleServer::notifyPeaks() {
    uint16_t peaks[4] = {
        static_cast<uint16_t>(lroundf(_sensor.minAngleDeg() * ANGLE_SCALE)),
        static_cast<uint16_t>(lroundf(_sensor.maxAngleDeg() * ANGLE_SCALE)),
        static_cast<uint16_t>(lroundf(_pan.minPanDeg() * ANGLE_SCALE)),
        static_cast<uint16_t>(lroundf(_pan.maxPanDeg() * ANGLE_SCALE)),
    };

    uint8_t payload[8];
    for (uint8_t i = 0; i < 4; i++) {
        payload[i * 2] = static_cast<uint8_t>(peaks[i] & 0xFF);
        payload[i * 2 + 1] = static_cast<uint8_t>(peaks[i] >> 8);
    }

    // setValue sempre: mantém o READ da characteristic válido para um cliente
    // que acabou de conectar e ainda não recebeu nenhum notify.
    peaksChar->setValue(payload, 8);

    // notify só quando muda. Extremo é grandeza que fica parada quase o tempo
    // todo — notificar a cada 200 ms gastaria banda BLE para repetir o mesmo
    // pacote, e é justamente durante uma captura de vibração (quando a fila
    // BLE está cheia) que isso mais atrapalharia.
    bool changed = !_hasSentPeaks;
    for (uint8_t i = 0; i < 4 && !changed; i++) {
        changed = peaks[i] != _lastSentPeaks[i];
    }
    if (!changed) {
        return;
    }
    for (uint8_t i = 0; i < 4; i++) {
        _lastSentPeaks[i] = peaks[i];
    }
    _hasSentPeaks = true;
    peaksChar->notify();
}

void BleServer::updateVibrationNotify() {
    VibrationCapture::Status status = _vibration.status();
    uint32_t now = millis();

    if (status == VibrationCapture::Status::Idle) {
        _lastReportedVibrationStatus = VibrationCapture::Status::Idle;
        return;
    }

    if (status == VibrationCapture::Status::Capturing) {
        // Throttled: sem isso, notify() seria chamado a cada iteração do
        // loop() (milhares de vezes/segundo) e sobrecarregaria a fila BLE.
        bool justStarted = _lastReportedVibrationStatus != VibrationCapture::Status::Capturing;
        if (justStarted || now - _lastVibrationStatusNotifyMs >= BLE_VIBRATION_STATUS_NOTIFY_INTERVAL_MS) {
            uint8_t payload[2] = {static_cast<uint8_t>(status), _vibration.progressPercent()};
            vibrationStatusChar->setValue(payload, 2);
            vibrationStatusChar->notify();
            _lastVibrationStatusNotifyMs = now;
        }
        _lastReportedVibrationStatus = status;
        return;
    }

    if (status == VibrationCapture::Status::Error) {
        if (_lastReportedVibrationStatus != VibrationCapture::Status::Error) {
            uint8_t payload[2] = {static_cast<uint8_t>(status), 0};
            vibrationStatusChar->setValue(payload, 2);
            vibrationStatusChar->notify();
            _lastReportedVibrationStatus = status;
        }
        return;
    }

    // status == Ready: primeiro esvazia os DOIS buffers de amostras (com
    // pacing entre pacotes) e só então notifica "pronto" na characteristic
    // de status. A ordem importa: o app (ble_source.py) considera a
    // captura concluída assim que recebe esse "pronto" — se ele chegasse
    // antes dos dados, o app leria um resultado truncado. Tilt primeiro,
    // pan depois, mas o "pronto" só sai quando os dois terminarem.
    uint16_t total = _vibration.sampleCount();
    if (_vibrationDataCursor < total || _vibrationPanDataCursor < total) {
        if (now - _lastVibrationChunkMs < BLE_VIBRATION_CHUNK_INTERVAL_MS) {
            return;
        }
        _lastVibrationChunkMs = now;

        if (_vibrationDataCursor < total) {
            sendVibrationChunk(vibrationDataChar, _vibrationDataCursor, total, false);
        } else {
            sendVibrationChunk(vibrationPanDataChar, _vibrationPanDataCursor, total, true);
        }
        return;
    }

    if (_lastReportedVibrationStatus != VibrationCapture::Status::Ready) {
        uint8_t payload[4] = {
            static_cast<uint8_t>(status),
            100,
            static_cast<uint8_t>(total & 0xFF),
            static_cast<uint8_t>(total >> 8),
        };
        vibrationStatusChar->setValue(payload, 4);
        vibrationStatusChar->notify();
        _lastReportedVibrationStatus = status;
    }
}

void BleServer::sendVibrationChunk(
    BLECharacteristic *characteristic, uint16_t &cursor, uint16_t total, bool pan
) {
    // Pacotes pequenos: o ATT MTU padrão do BLE (sem negociação) só garante
    // 20 bytes úteis por notificação — 2 bytes de índice + 8 amostras (16
    // bytes) = 18 bytes, cabe mesmo sem MTU maior negociado.
    constexpr uint16_t kSamplesPerPacket = 8;
    uint16_t remaining = total - cursor;
    uint16_t chunk = remaining < kSamplesPerPacket ? remaining : kSamplesPerPacket;

    uint8_t payload[2 + kSamplesPerPacket * 2];
    payload[0] = static_cast<uint8_t>(cursor & 0xFF);
    payload[1] = static_cast<uint8_t>(cursor >> 8);
    for (uint16_t i = 0; i < chunk; i++) {
        int16_t sample = pan ? _vibration.panSampleAt(cursor + i) : _vibration.sampleAt(cursor + i);
        payload[2 + i * 2] = static_cast<uint8_t>(sample & 0xFF);
        payload[2 + i * 2 + 1] = static_cast<uint8_t>((sample >> 8) & 0xFF);
    }
    characteristic->setValue(payload, 2 + chunk * 2);
    characteristic->notify();
    cursor += chunk;
}

void BleServer::update() {
    if (restartAdvertisingPending) {
        restartAdvertisingPending = false;
        BLEDevice::startAdvertising();
    }
    if (_calibratePending) {
        _calibratePending = false;
        // Uma ação de calibração zera os DOIS eixos: é o que os apps expõem
        // como um único botão "Calibrar".
        _sensor.calibrate();
        _pan.calibrate();
    }
    if (_resetPeaksPending) {
        _resetPeaksPending = false;
        _sensor.resetPeaks();
        _pan.resetPeaks();
    }
    consumeResend(_resendTiltFrom, _vibrationDataCursor);
    consumeResend(_resendPanFrom, _vibrationPanDataCursor);
    notifyAngles();
    updateVibrationNotify();
}
