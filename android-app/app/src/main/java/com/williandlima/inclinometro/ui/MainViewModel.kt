package com.williandlima.inclinometro.ui

import android.app.Application
import android.content.Context
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.williandlima.inclinometro.datasource.AngleDataSource
import com.williandlima.inclinometro.datasource.AngleReading
import com.williandlima.inclinometro.datasource.BleAngleDataSource
import com.williandlima.inclinometro.datasource.BleConnectionTester
import com.williandlima.inclinometro.datasource.BleContract
import com.williandlima.inclinometro.datasource.BleScanner
import com.williandlima.inclinometro.datasource.ConnectionMode
import com.williandlima.inclinometro.datasource.SimulatedAngleDataSource
import com.williandlima.inclinometro.limits.HistoryRepository
import com.williandlima.inclinometro.limits.LimitAxis
import com.williandlima.inclinometro.limits.LimitKind
import com.williandlima.inclinometro.limits.LimitTracker
import com.williandlima.inclinometro.limits.VibrationStats
import com.williandlima.inclinometro.limits.VibrationStatsCalculator
import com.williandlima.inclinometro.limits.db.AppDatabase
import com.williandlima.inclinometro.report.PdfReportGenerator
import java.io.File
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.Job
import kotlinx.coroutines.TimeoutCancellationException
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.catch
import kotlinx.coroutines.flow.collect
import kotlinx.coroutines.flow.launchIn
import kotlinx.coroutines.flow.onEach
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.launch
import kotlinx.coroutines.withTimeoutOrNull

/** Estado da conexão exibido na UI, mesma semântica do app desktop
 * (`_CONN_STYLES` em `python-app/ui/main_window.py`). */
enum class ConnectionStatus { PARADO, CONECTANDO, CONECTADO, ERRO, SIMULACAO }

// Exibição da leitura contínua: degrau por eixo (0,25° padrão ou 0,1°), com
// histerese — a regra mora em AngleDisplay.kt, a mesma do app desktop. Nada
// disso afeta os dados guardados — histórico, mín/máx e relatório usam sempre
// o ângulo bruto (`currentAngle`).

// Onde a resolução de exibição fica guardada entre sessões.
private const val PREFS_NAME = "inclinometro"
private const val PREF_TILT_STEP = "exibicao_tilt_step_deg"
private const val PREF_PAN_STEP = "exibicao_pan_step_deg"

data class UiState(
    val mode: ConnectionMode = ConnectionMode.SIMULATED,
    val running: Boolean = false,
    val currentAngle: Double? = null,   // valor bruto da leitura (tilt)
    val displayAngle: Double? = null,   // valor exibido (degrau do eixo com histerese)
    val minReading: AngleReading? = null,
    val maxReading: AngleReading? = null,
    val minFlash: Boolean = false,
    val maxFlash: Boolean = false,
    // Eixo de azimute (pan) — mesma estrutura do tilt. `panAvailable` fica
    // false quando o ESP32 conectado tem firmware anterior à v1.2.0 e não
    // mede azimute; a UI então mostra o eixo como sem dado em vez de zero.
    val currentPan: Double? = null,
    val displayPan: Double? = null,
    val panMinReading: AngleReading? = null,
    val panMaxReading: AngleReading? = null,
    val panMinFlash: Boolean = false,
    val panMaxFlash: Boolean = false,
    val panAvailable: Boolean = true,
    val statusMessage: String = "Pronto.",
    val bleDeviceAddress: String = "",
    val connectionStatus: ConnectionStatus = ConnectionStatus.PARADO,
    // Versão do firmware do ESP32 conectado (null enquanto não lida / simulação).
    val firmwareVersion: String? = null,
    val firmwareOutdated: Boolean = false,
    // Degrau de exibição de cada eixo (0,25° padrão; 0,1° para bancada com
    // referência de precisão). Trocável no cartão de cada eixo e em
    // Configurações, e lembrado entre sessões. Cada eixo tem o seu.
    val tiltDisplayStepDeg: Double = AngleDisplay.DEFAULT_STEP_DEG,
    val panDisplayStepDeg: Double = AngleDisplay.DEFAULT_STEP_DEG,
    val scanning: Boolean = false,
    val scanResults: List<BleScanner.Found> = emptyList(),
    val bleTestInProgress: Boolean = false,
    val bleTestResult: String? = null,
    val bleTestSuccess: Boolean = false,
    val calibrating: Boolean = false,
    val vibrationCapturing: Boolean = false,
    val vibrationProgress: Int = 0,
    val vibrationResult: VibrationStats? = null,
    // Nulo quando a captura veio de firmware anterior à v1.3.0, que não
    // amostra azimute no Modo Vibração.
    val vibrationPanResult: VibrationStats? = null,
)

class MainViewModel(application: Application) : AndroidViewModel(application) {

    private val repository = HistoryRepository(AppDatabase.getInstance(application).dao())
    // Um rastreador de mín/máx por eixo: os extremos são independentes.
    private val tracker = LimitTracker(LimitAxis.TILT)
    private val panTracker = LimitTracker(LimitAxis.PAN)

    private val prefs = application.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)

    private val _uiState = MutableStateFlow(
        UiState(
            tiltDisplayStepDeg = AngleDisplay.normalize(
                prefs.getFloat(PREF_TILT_STEP, AngleDisplay.DEFAULT_STEP_DEG.toFloat()).toDouble(),
            ),
            panDisplayStepDeg = AngleDisplay.normalize(
                prefs.getFloat(PREF_PAN_STEP, AngleDisplay.DEFAULT_STEP_DEG.toFloat()).toDouble(),
            ),
        ),
    )
    val uiState: StateFlow<UiState> = _uiState.asStateFlow()

    private var collectJob: Job? = null
    private var vibrationJob: Job? = null
    private var scanJob: Job? = null
    private var testJob: Job? = null
    private var currentSessionId: Long? = null
    private var currentSource: AngleDataSource? = null

    private var lastVibrationCaptureId: Long? = null
    private var lastVibrationReadings: List<AngleReading> = emptyList()
    private var lastVibrationRateHz: Int = 0

    fun setMode(mode: ConnectionMode) {
        if (!_uiState.value.running) {
            _uiState.update { it.copy(mode = mode) }
        }
    }

    fun setBleDeviceAddress(address: String) {
        _uiState.update { it.copy(bleDeviceAddress = address) }
    }

    // ---------------------------------------------------------- scan BLE

    fun scanBleDevices() {
        if (_uiState.value.scanning) return
        _uiState.update { it.copy(scanning = true, scanResults = emptyList(), statusMessage = "Escaneando dispositivos BLE...") }
        scanJob = viewModelScope.launch {
            val found = linkedMapOf<String, BleScanner.Found>()
            try {
                withTimeoutOrNull(BLE_SCAN_TIMEOUT_MS) {
                    BleScanner.scan(getApplication()).collect { result ->
                        found[result.address] = result
                        _uiState.update { it.copy(scanResults = found.values.toList()) }
                    }
                }
                _uiState.update {
                    it.copy(
                        statusMessage = if (found.isEmpty()) {
                            "Nenhum dispositivo BLE encontrado por perto."
                        } else {
                            "${found.size} dispositivo(s) encontrado(s)."
                        },
                    )
                }
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                _uiState.update { it.copy(statusMessage = "Falha ao escanear: ${e.message}") }
            } finally {
                _uiState.update { it.copy(scanning = false) }
            }
        }
    }

    fun cancelBleScan() {
        scanJob?.cancel()
    }

    // ------------------------------------------------------ testar conexão

    fun testBleConnection() {
        val address = _uiState.value.bleDeviceAddress.trim()
        if (address.isBlank()) {
            _uiState.update {
                it.copy(bleTestResult = "Informe ou escaneie um endereço MAC primeiro.", bleTestSuccess = false)
            }
            return
        }
        if (_uiState.value.bleTestInProgress) return
        _uiState.update { it.copy(bleTestInProgress = true, bleTestResult = "Testando...", bleTestSuccess = false) }
        testJob = viewModelScope.launch {
            try {
                val result = BleConnectionTester.test(getApplication(), address)
                val panText = result.panDeg
                    ?.let { pan -> ", azimute %.2f°".format(pan) }
                    ?: ", sem eixo de azimute"
                _uiState.update {
                    it.copy(
                        bleTestResult = "✓ ESP32 respondeu — inclinação %.2f°%s (firmware v%s)".format(
                            result.angleDeg,
                            panText,
                            result.firmwareVersion,
                        ),
                        bleTestSuccess = true,
                    )
                }
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                _uiState.update { it.copy(bleTestResult = "✗ Falha: ${e.message}", bleTestSuccess = false) }
            } finally {
                _uiState.update { it.copy(bleTestInProgress = false) }
            }
        }
    }

    fun toggleStartStop() {
        if (_uiState.value.running) stop() else start()
    }

    private fun start() {
        val mode = _uiState.value.mode
        if (mode == ConnectionMode.REAL && _uiState.value.bleDeviceAddress.isBlank()) {
            _uiState.update { it.copy(statusMessage = "Informe o endereço MAC do dispositivo BLE.") }
            return
        }

        val source: AngleDataSource = if (mode == ConnectionMode.SIMULATED) {
            SimulatedAngleDataSource()
        } else {
            BleAngleDataSource(getApplication(), _uiState.value.bleDeviceAddress)
        }
        currentSource = source

        tracker.reset()
        panTracker.reset()
        viewModelScope.launch {
            currentSessionId = repository.startSession(mode)
            _uiState.update {
                it.copy(
                    running = true,
                    minReading = null,
                    maxReading = null,
                    displayAngle = null,
                    panMinReading = null,
                    panMaxReading = null,
                    displayPan = null,
                    panAvailable = true,
                    firmwareVersion = null,
                    firmwareOutdated = false,
                    statusMessage = "Conectado: ${source.label}",
                    connectionStatus = if (mode == ConnectionMode.SIMULATED) {
                        ConnectionStatus.SIMULACAO
                    } else {
                        ConnectionStatus.CONECTANDO
                    },
                )
            }
            collectJob = source.readings()
                .catch { e ->
                    _uiState.update { s ->
                        s.copy(
                            statusMessage = "Erro: ${e.message}",
                            connectionStatus = if (s.mode == ConnectionMode.REAL) ConnectionStatus.ERRO else s.connectionStatus,
                        )
                    }
                }
                .onEach { reading -> onReading(reading) }
                .launchIn(viewModelScope)
        }
    }

    /** Arredonda para o degrau de exibição, com histerese para não alternar
     * entre dois degraus quando o valor fica na fronteira. */
    private fun angleForDisplay(angleDeg: Double, current: Double?, step: Double): Double =
        AngleDisplay.forDisplay(angleDeg, current, step)

    /**
     * Troca a resolução de exibição da inclinação (cartão do eixo ou
     * Configurações), grava para a próxima sessão e redesenha na hora — o
     * valor exibido estava no degrau antigo, então a histerese recomeça.
     */
    fun setTiltDisplayStep(stepDeg: Double) {
        val step = AngleDisplay.normalize(stepDeg)
        prefs.edit().putFloat(PREF_TILT_STEP, step.toFloat()).apply()
        _uiState.update {
            it.copy(tiltDisplayStepDeg = step, displayAngle = it.currentAngle?.let { v -> angleForDisplay(v, null, step) })
        }
    }

    /** Troca a resolução de exibição do azimute — ver [setTiltDisplayStep]. */
    fun setPanDisplayStep(stepDeg: Double) {
        val step = AngleDisplay.normalize(stepDeg)
        prefs.edit().putFloat(PREF_PAN_STEP, step.toFloat()).apply()
        _uiState.update {
            it.copy(panDisplayStepDeg = step, displayPan = it.currentPan?.let { v -> angleForDisplay(v, null, step) })
        }
    }

    private suspend fun onReading(reading: AngleReading) {
        val sessionId = currentSessionId ?: return
        repository.addReading(sessionId, reading)
        val pan = reading.panDeg
        val firmware = currentSource?.firmwareVersion
        _uiState.update {
            it.copy(
                firmwareVersion = firmware,
                firmwareOutdated = BleContract.firmwareOutdated(firmware),
                currentAngle = reading.angleDeg,
                displayAngle = angleForDisplay(reading.angleDeg, it.displayAngle, it.tiltDisplayStepDeg),
                currentPan = pan,
                displayPan = if (pan == null) null else angleForDisplay(pan, it.displayPan, it.panDisplayStepDeg),
                panAvailable = pan != null,
                connectionStatus = if (it.mode == ConnectionMode.REAL) ConnectionStatus.CONECTADO else it.connectionStatus,
            )
        }

        for (event in tracker.process(reading)) {
            repository.addLimitEvent(sessionId, event)
            when (event.kind) {
                LimitKind.MIN -> {
                    _uiState.update { it.copy(minReading = event.reading, minFlash = true) }
                    scheduleFlashReset(LimitAxis.TILT, isMin = true)
                }
                LimitKind.MAX -> {
                    _uiState.update { it.copy(maxReading = event.reading, maxFlash = true) }
                    scheduleFlashReset(LimitAxis.TILT, isMin = false)
                }
            }
        }

        // Devolve lista vazia quando a leitura não traz azimute, então este
        // laço simplesmente não roda com firmware anterior à v1.2.0.
        for (event in panTracker.process(reading)) {
            repository.addLimitEvent(sessionId, event)
            when (event.kind) {
                LimitKind.MIN -> {
                    _uiState.update { it.copy(panMinReading = event.reading, panMinFlash = true) }
                    scheduleFlashReset(LimitAxis.PAN, isMin = true)
                }
                LimitKind.MAX -> {
                    _uiState.update { it.copy(panMaxReading = event.reading, panMaxFlash = true) }
                    scheduleFlashReset(LimitAxis.PAN, isMin = false)
                }
            }
        }
    }

    private fun scheduleFlashReset(axis: LimitAxis, isMin: Boolean) {
        viewModelScope.launch {
            delay(1500)
            _uiState.update {
                when {
                    axis == LimitAxis.PAN && isMin -> it.copy(panMinFlash = false)
                    axis == LimitAxis.PAN -> it.copy(panMaxFlash = false)
                    isMin -> it.copy(minFlash = false)
                    else -> it.copy(maxFlash = false)
                }
            }
        }
    }

    private fun stop() {
        collectJob?.cancel()
        collectJob = null
        vibrationJob?.cancel()
        vibrationJob = null
        currentSource = null
        val sessionId = currentSessionId
        viewModelScope.launch {
            sessionId?.let { repository.endSession(it) }
            _uiState.update {
                it.copy(
                    running = false,
                    displayAngle = null,
                    displayPan = null,
                    firmwareVersion = null,
                    firmwareOutdated = false,
                    statusMessage = "Parado.",
                    connectionStatus = ConnectionStatus.PARADO,
                )
            }
        }
    }

    fun resetLimits() {
        val wasRunning = _uiState.value.running
        val mode = _uiState.value.mode
        viewModelScope.launch {
            // Com firmware v1.5.0+ quem guarda os extremos é o ESP32 (ver
            // [LimitTracker]). Zerar só o lado do app não adiantaria: a
            // próxima notificação traria os extremos antigos de volta.
            //
            // A ordem importa e é esta: dispositivo primeiro, rastreadores
            // depois. Invertida, uma leitura chegando durante a escrita BLE
            // (que suspende) reintroduziria os extremos antigos — e, como
            // mín/máx só andam para fora, eles ficariam.
            val source = currentSource
            if (source != null && source.supportsPeakReset) {
                runCatching { source.resetPeaks() }
            }
            tracker.reset()
            panTracker.reset()

            if (wasRunning) {
                currentSessionId?.let { repository.endSession(it) }
                currentSessionId = repository.startSession(mode)
            }
            _uiState.update {
                it.copy(
                    minReading = null,
                    maxReading = null,
                    panMinReading = null,
                    panMaxReading = null,
                    statusMessage = if (wasRunning) {
                        "Limites resetados (histórico anterior preservado)."
                    } else {
                        "Limites resetados."
                    },
                )
            }
        }
    }

    suspend fun generateReportFile(): File? = reportOrNull {
        val sessionId = currentSessionId ?: repository.listSessions().firstOrNull()?.id ?: run {
            _uiState.update { it.copy(statusMessage = "Ainda não há nenhuma sessão registrada.") }
            return@reportOrNull null
        }
        val session = repository.getSession(sessionId) ?: return@reportOrNull null
        val readings = repository.getReadings(sessionId)
        val events = repository.getLimitEvents(sessionId)
        PdfReportGenerator.generate(getApplication(), session, readings, events)
    }

    /**
     * Roda a geração de um relatório; uma falha (ex.: armazenamento
     * indisponível) vira mensagem de status em vez de derrubar o app — a
     * chamada vem de uma coroutine da tela, sem tratamento de exceção.
     */
    private suspend fun reportOrNull(block: suspend () -> File?): File? =
        try {
            block()
        } catch (e: CancellationException) {
            throw e
        } catch (e: Exception) {
            _uiState.update { it.copy(statusMessage = "Falha ao gerar relatório: ${e.message}") }
            null
        }

    // ------------------------------------------------------------ calibração

    fun calibrate() {
        val source = currentSource
        if (!_uiState.value.running || source == null) {
            _uiState.update { it.copy(statusMessage = "Inicie a leitura antes de calibrar.") }
            return
        }
        if (!source.supportsCalibration) {
            _uiState.update { it.copy(statusMessage = "Esta fonte de dados não suporta calibração.") }
            return
        }

        _uiState.update { it.copy(calibrating = true) }
        viewModelScope.launch {
            try {
                source.calibrate()
                _uiState.update { it.copy(statusMessage = "Calibração concluída: posição atual definida como 0°.") }
                resetLimits()
            } catch (e: Exception) {
                _uiState.update { it.copy(statusMessage = "Falha na calibração: ${e.message}") }
            } finally {
                _uiState.update { it.copy(calibrating = false) }
            }
        }
    }

    // ------------------------------------------------------- Modo Vibração

    fun startVibrationCapture(durationS: Int, rateHz: Int) {
        val source = currentSource
        if (!_uiState.value.running || source == null) {
            _uiState.update { it.copy(statusMessage = "Inicie a leitura antes de usar o Modo Vibração.") }
            return
        }
        if (!source.supportsVibrationCapture) {
            _uiState.update { it.copy(statusMessage = "Esta fonte de dados não suporta captura de vibração.") }
            return
        }

        val mode = _uiState.value.mode
        _uiState.update {
            it.copy(
                vibrationCapturing = true,
                vibrationProgress = 0,
                vibrationResult = null,
                statusMessage = "Captura de vibração em andamento...",
            )
        }
        vibrationJob = viewModelScope.launch {
            try {
                val readings = source.startVibrationCapture(durationS, rateHz) { percent ->
                    _uiState.update { it.copy(vibrationProgress = percent) }
                }
                if (readings.isEmpty()) {
                    _uiState.update { it.copy(statusMessage = "Nenhuma amostra foi capturada.") }
                    return@launch
                }

                val captureId = repository.saveVibrationCapture(mode, durationS, rateHz, readings)
                val tilt = VibrationStatsCalculator.analyzeAxis(readings, rateHz, LimitAxis.TILT)
                val pan = if (VibrationStatsCalculator.hasPanSamples(readings)) {
                    VibrationStatsCalculator.analyzeAxis(readings, rateHz, LimitAxis.PAN)
                } else {
                    null
                }

                lastVibrationCaptureId = captureId
                lastVibrationReadings = readings
                lastVibrationRateHz = rateHz

                _uiState.update {
                    it.copy(
                        vibrationResult = tilt.stats,
                        vibrationPanResult = pan?.stats,
                        statusMessage = "Captura de vibração concluída.",
                    )
                }
            } catch (e: TimeoutCancellationException) {
                _uiState.update { it.copy(statusMessage = "Tempo esgotado aguardando a captura de vibração.") }
            } catch (e: CancellationException) {
                _uiState.update { it.copy(statusMessage = "Captura de vibração cancelada.") }
            } catch (e: Exception) {
                _uiState.update { it.copy(statusMessage = "Captura de vibração: ${e.message}") }
            } finally {
                _uiState.update { it.copy(vibrationCapturing = false) }
            }
        }
    }

    fun cancelVibrationCapture() {
        vibrationJob?.cancel()
    }

    fun dismissVibrationResult() {
        _uiState.update { it.copy(vibrationResult = null, vibrationPanResult = null) }
    }

    suspend fun generateVibrationReportFile(): File? = reportOrNull {
        val captureId = lastVibrationCaptureId ?: return@reportOrNull null
        val readings = lastVibrationReadings
        if (readings.isEmpty()) return@reportOrNull null
        val captureInfo = repository.listVibrationCaptures().firstOrNull { it.id == captureId }
            ?: return@reportOrNull null
        val rateHz = lastVibrationRateHz.coerceAtLeast(1)
        val tilt = VibrationStatsCalculator.analyzeAxis(readings, rateHz, LimitAxis.TILT)
        val pan = if (VibrationStatsCalculator.hasPanSamples(readings)) {
            VibrationStatsCalculator.analyzeAxis(readings, rateHz, LimitAxis.PAN)
        } else {
            null
        }
        PdfReportGenerator.generateVibrationReport(
            getApplication(),
            captureInfo,
            readings,
            tilt,
            pan,
        )
    }

    override fun onCleared() {
        collectJob?.cancel()
        vibrationJob?.cancel()
        scanJob?.cancel()
        testJob?.cancel()
        super.onCleared()
    }

    private companion object {
        // Mesma duração usada como default em ble_source.py (scan_devices).
        const val BLE_SCAN_TIMEOUT_MS = 5000L
    }
}
