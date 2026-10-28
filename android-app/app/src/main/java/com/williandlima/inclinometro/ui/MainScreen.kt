package com.williandlima.inclinometro.ui

import android.content.Context
import android.content.Intent
import android.widget.Toast
import androidx.compose.animation.animateColorAsState
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.RadioButton
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.core.content.FileProvider
import com.williandlima.inclinometro.datasource.BleScanner
import com.williandlima.inclinometro.datasource.BleContract
import com.williandlima.inclinometro.datasource.ConnectionMode
import com.williandlima.inclinometro.limits.VibrationStats
import com.williandlima.inclinometro.ui.theme.AmberFlash
import com.williandlima.inclinometro.ui.theme.Green
import com.williandlima.inclinometro.ui.theme.Orange
import com.williandlima.inclinometro.ui.theme.Red
import java.io.File
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import kotlinx.coroutines.launch

private val timeFormatter = SimpleDateFormat("HH:mm:ss", Locale("pt", "BR"))

@Composable
fun MainScreen(viewModel: MainViewModel) {
    val state by viewModel.uiState.collectAsState()
    val context = LocalContext.current
    val scope = rememberCoroutineScope()

    var showVibrationConfig by remember { mutableStateOf(false) }

    val minCardColor by animateColorAsState(
        targetValue = if (state.minFlash) AmberFlash else MaterialTheme.colorScheme.surfaceVariant,
        label = "minCardColor",
    )
    val maxCardColor by animateColorAsState(
        targetValue = if (state.maxFlash) AmberFlash else MaterialTheme.colorScheme.surfaceVariant,
        label = "maxCardColor",
    )
    val panMinCardColor by animateColorAsState(
        targetValue = if (state.panMinFlash) AmberFlash else MaterialTheme.colorScheme.surfaceVariant,
        label = "panMinCardColor",
    )
    val panMaxCardColor by animateColorAsState(
        targetValue = if (state.panMaxFlash) AmberFlash else MaterialTheme.colorScheme.surfaceVariant,
        label = "panMaxCardColor",
    )

    Column(
        modifier = Modifier
            .fillMaxSize()
            // Rolável: com os dois eixos empilhados, o conteúdo passa da
            // altura da tela em celulares menores.
            .verticalScroll(rememberScrollState())
            .padding(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        BrandHeader()
        Spacer(Modifier.height(8.dp))

        val modoLabel = if (state.mode == ConnectionMode.SIMULATED) "Simulação" else "Real (BLE)"
        val estadoLabel = if (state.running) "em execução" else "parado"
        Text("Modo: $modoLabel — $estadoLabel", style = MaterialTheme.typography.bodyMedium)

        Spacer(Modifier.height(8.dp))
        ConnectionBadge(state.connectionStatus)

        // Versões do app e do firmware conectado; firmware mais antigo que o
        // recomendado aparece em laranja, com o aviso para regravar.
        val appVersion = remember(context) {
            runCatching { context.packageManager.getPackageInfo(context.packageName, 0).versionName }
                .getOrNull() ?: "?"
        }
        val firmwareText = when {
            !state.running -> "Firmware: —"
            state.mode == ConnectionMode.SIMULATED -> "Firmware: simulação"
            state.firmwareVersion == null -> "Firmware: lendo..."
            state.firmwareOutdated -> "Firmware v${state.firmwareVersion} — desatualizado, grave a " +
                BleContract.decodeFirmwareVersion(BleContract.MIN_RECOMMENDED_FIRMWARE_CODE)
            else -> "Firmware v${state.firmwareVersion}"
        }
        Spacer(Modifier.height(4.dp))
        Text(
            "App v$appVersion  ·  $firmwareText",
            style = MaterialTheme.typography.bodySmall,
            color = if (state.firmwareOutdated) Orange else Color.Gray,
            fontWeight = if (state.firmwareOutdated) FontWeight.Bold else null,
        )

        Spacer(Modifier.height(16.dp))

        AxisPanel(
            title = "Inclinação (tilt)",
            displayValue = state.displayAngle,
            displayStepDeg = state.tiltDisplayStepDeg,
            onDisplayStepChange = viewModel::setTiltDisplayStep,
            note = null,
            minValue = state.minReading?.angleDeg,
            minTimestamp = state.minReading?.timestamp,
            maxValue = state.maxReading?.angleDeg,
            maxTimestamp = state.maxReading?.timestamp,
            minCardColor = minCardColor,
            maxCardColor = maxCardColor,
        )

        Spacer(Modifier.height(20.dp))

        AxisPanel(
            title = "Azimute (pan)",
            displayValue = state.displayPan,
            displayStepDeg = state.panDisplayStepDeg,
            onDisplayStepChange = viewModel::setPanDisplayStep,
            // Deixa explícito que o eixo está sem dado, em vez de o usuário
            // achar que o azimute está travado em zero.
            note = if (state.running && !state.panAvailable) "firmware sem este eixo" else null,
            minValue = state.panMinReading?.panDeg,
            minTimestamp = state.panMinReading?.timestamp,
            maxValue = state.panMaxReading?.panDeg,
            maxTimestamp = state.panMaxReading?.timestamp,
            minCardColor = panMinCardColor,
            maxCardColor = panMaxCardColor,
        )

        Spacer(Modifier.height(24.dp))

        if (!state.running) {
            ModeSelector(
                mode = state.mode,
                onModeChange = viewModel::setMode,
                tiltDisplayStepDeg = state.tiltDisplayStepDeg,
                onTiltDisplayStepChange = viewModel::setTiltDisplayStep,
                panDisplayStepDeg = state.panDisplayStepDeg,
                onPanDisplayStepChange = viewModel::setPanDisplayStep,
                bleAddress = state.bleDeviceAddress,
                onBleAddressChange = viewModel::setBleDeviceAddress,
                scanning = state.scanning,
                scanResults = state.scanResults,
                onScanClick = viewModel::scanBleDevices,
                onScanResultSelected = { address ->
                    viewModel.setBleDeviceAddress(address)
                    viewModel.cancelBleScan()
                },
                onScanDialogDismiss = viewModel::cancelBleScan,
                testInProgress = state.bleTestInProgress,
                testResult = state.bleTestResult,
                testSuccess = state.bleTestSuccess,
                onTestConnectionClick = viewModel::testBleConnection,
            )
            Spacer(Modifier.height(16.dp))
        }

        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = viewModel::toggleStartStop) {
                Text(if (state.running) "Parar" else "Iniciar")
            }
            OutlinedButton(onClick = viewModel::resetLimits) {
                Text("Resetar limites")
            }
        }

        Spacer(Modifier.height(8.dp))

        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            OutlinedButton(onClick = viewModel::calibrate, enabled = state.running && !state.calibrating) {
                Text(if (state.calibrating) "Calibrando..." else "Calibrar")
            }
            OutlinedButton(
                onClick = { showVibrationConfig = true },
                enabled = state.running && !state.vibrationCapturing,
            ) {
                Text("Modo Vibração")
            }
        }

        Spacer(Modifier.height(8.dp))

        Button(onClick = {
            scope.launch {
                val file = viewModel.generateReportFile()
                if (file != null) sharePdf(context, file)
            }
        }) {
            Text("Gerar relatório PDF")
        }

        Spacer(Modifier.height(16.dp))
        Text(state.statusMessage, style = MaterialTheme.typography.bodySmall)
    }

    if (showVibrationConfig) {
        VibrationConfigDialog(
            onConfirm = { durationS, rateHz ->
                showVibrationConfig = false
                viewModel.startVibrationCapture(durationS, rateHz)
            },
            onDismiss = { showVibrationConfig = false },
        )
    }

    if (state.vibrationCapturing) {
        VibrationProgressDialog(
            progress = state.vibrationProgress,
            onCancel = viewModel::cancelVibrationCapture,
        )
    }

    state.vibrationResult?.let { stats ->
        VibrationResultDialog(
            stats = stats,
            panStats = state.vibrationPanResult,
            onDismiss = viewModel::dismissVibrationResult,
            onSaveReport = {
                scope.launch {
                    val file = viewModel.generateVibrationReportFile()
                    viewModel.dismissVibrationResult()
                    if (file != null) sharePdf(context, file)
                }
            },
        )
    }
}

@Composable
private fun BrandHeader() {
    val context = LocalContext.current
    val logoResId = remember(context) { context.resources.getIdentifier("logo", "drawable", context.packageName) }

    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text("Inclinômetro", style = MaterialTheme.typography.titleLarge, fontWeight = FontWeight.Bold)
        if (logoResId != 0) {
            Image(
                painter = painterResource(id = logoResId),
                contentDescription = null,
                modifier = Modifier.height(40.dp),
            )
        } else {
            Text("AVIBRAS aeroco", color = Orange, fontWeight = FontWeight.Bold)
        }
    }
}

private data class ConnBadgeStyle(val text: String, val background: Color?, val textColor: Color)

@Composable
private fun ConnectionBadge(status: ConnectionStatus) {
    val style = when (status) {
        ConnectionStatus.PARADO -> ConnBadgeStyle("○ Parado", null, Color.Gray)
        ConnectionStatus.CONECTANDO -> ConnBadgeStyle("◐ Conectando...", null, Orange)
        ConnectionStatus.CONECTADO -> ConnBadgeStyle("● Conectado", Green, Color.White)
        ConnectionStatus.ERRO -> ConnBadgeStyle("● Falha de conexão", Red, Color.White)
        ConnectionStatus.SIMULACAO -> ConnBadgeStyle("● Simulação (interna)", null, Orange)
    }
    val modifier = if (style.background != null) {
        Modifier
            .background(style.background, RoundedCornerShape(10.dp))
            .padding(horizontal = 12.dp, vertical = 4.dp)
    } else {
        Modifier
    }
    Text(style.text, color = style.textColor, fontWeight = FontWeight.Bold, modifier = modifier)
}

@Composable
private fun VibrationConfigDialog(
    onConfirm: (durationS: Int, rateHz: Int) -> Unit,
    onDismiss: () -> Unit,
) {
    var durationText by remember { mutableStateOf("30") }
    var rateText by remember { mutableStateOf("50") }

    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Modo Vibração — Configurar captura") },
        text = {
            Column {
                Text(
                    "Posicione o pan-tilt na posição de referência e use \"Calibrar\" " +
                        "antes de iniciar, para que a variação registrada seja relativa a essa posição.",
                    style = MaterialTheme.typography.bodySmall,
                )
                Spacer(Modifier.height(12.dp))
                OutlinedTextField(
                    value = durationText,
                    onValueChange = { durationText = it.filter(Char::isDigit) },
                    label = { Text("Duração da captura (s)") },
                )
                Spacer(Modifier.height(8.dp))
                OutlinedTextField(
                    value = rateText,
                    onValueChange = { rateText = it.filter(Char::isDigit) },
                    label = { Text("Taxa de amostragem (Hz)") },
                )
            }
        },
        confirmButton = {
            TextButton(onClick = {
                val durationS = durationText.toIntOrNull()?.coerceIn(5, 600) ?: 30
                val rateHz = rateText.toIntOrNull()?.coerceIn(1, 200) ?: 50
                onConfirm(durationS, rateHz)
            }) {
                Text("Iniciar")
            }
        },
        dismissButton = { TextButton(onClick = onDismiss) { Text("Cancelar") } },
    )
}

@Composable
private fun VibrationProgressDialog(progress: Int, onCancel: () -> Unit) {
    AlertDialog(
        onDismissRequest = { /* não fecha ao tocar fora enquanto captura */ },
        title = { Text("Modo Vibração") },
        text = {
            Column {
                Text("Capturando... $progress%")
                Spacer(Modifier.height(8.dp))
                LinearProgressIndicator(
                    progress = { progress / 100f },
                    modifier = Modifier.fillMaxWidth(),
                )
            }
        },
        confirmButton = { TextButton(onClick = onCancel) { Text("Cancelar") } },
    )
}

/** Bloco de estatísticas de um eixo dentro do diálogo de resultado. */
@Composable
private fun VibrationAxisSummary(title: String, stats: VibrationStats) {
    Text(title, fontWeight = FontWeight.Bold, color = Orange)
    Text("Desvio padrão: %.3f°   RMS: %.3f°".format(stats.stdDevDeg, stats.rmsDeg))
    Text("Pico a pico: %.3f°".format(stats.peakToPeakDeg))
    Text("Mínimo / Máximo: %.3f° / %.3f°".format(stats.minDeg, stats.maxDeg))
    val dominantFreq = stats.dominantFreqHz
    if (dominantFreq != null) {
        Text(
            "Frequência dominante: %.2f Hz (amplitude %.3f°, SNR %.1f dB)".format(
                dominantFreq,
                stats.dominantAmplitudeDeg ?: 0.0,
                stats.dominantSnrDb ?: 0.0,
            )
        )
    } else {
        Text("Frequência dominante: nenhum pico confiável (sinal compatível com ruído).")
    }
}

@Composable
private fun VibrationResultDialog(
    stats: VibrationStats,
    panStats: VibrationStats?,
    onDismiss: () -> Unit,
    onSaveReport: () -> Unit,
) {
    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Modo Vibração — Resultado da captura") },
        text = {
            // Rolável: com os dois eixos, o resumo passa da altura útil de um
            // AlertDialog em telas menores.
            Column(modifier = Modifier.verticalScroll(rememberScrollState())) {
                Text("Amostras: ${stats.nSamples} (duração efetiva: %.2f s)".format(stats.durationS))
                Spacer(modifier = Modifier.height(12.dp))
                VibrationAxisSummary("Inclinação (tilt)", stats)
                Spacer(modifier = Modifier.height(12.dp))
                if (panStats != null) {
                    VibrationAxisSummary("Azimute (pan)", panStats)
                } else {
                    Text("Azimute (pan)", fontWeight = FontWeight.Bold, color = Orange)
                    Text("não capturado — firmware anterior à v1.3.0.")
                }
            }
        },
        confirmButton = { TextButton(onClick = onSaveReport) { Text("Salvar relatório PDF") } },
        dismissButton = { TextButton(onClick = onDismiss) { Text("Fechar") } },
    )
}

/**
 * Painel de um eixo: título, valor grande em tempo real, seletor de resolução
 * da tela (0,25°/0,1°, o mesmo de Configurações, à mão também durante a
 * leitura) e o par de caixas de mínimo/máximo. Os dois eixos usam o mesmo composable, empilhados — lado a
 * lado não caberia com legibilidade na largura de um celular.
 */
@Composable
private fun AxisPanel(
    title: String,
    displayValue: Double?,
    displayStepDeg: Double,
    onDisplayStepChange: (Double) -> Unit,
    note: String?,
    minValue: Double?,
    minTimestamp: Long?,
    maxValue: Double?,
    maxTimestamp: Long?,
    minCardColor: Color,
    maxCardColor: Color,
) {
    Text(title, style = MaterialTheme.typography.titleMedium, color = Orange, fontWeight = FontWeight.Bold)
    Text(
        text = displayValue?.let { AngleDisplay.format(it, displayStepDeg) } ?: "--.--°",
        style = MaterialTheme.typography.displayMedium,
        fontWeight = FontWeight.Bold,
    )
    Row(verticalAlignment = Alignment.CenterVertically) {
        Text("Resolução:", style = MaterialTheme.typography.bodySmall, color = Color.Gray)
        for (option in AngleDisplay.STEPS_DEG) {
            RadioButton(selected = displayStepDeg == option, onClick = { onDisplayStepChange(option) })
            Text(AngleDisplay.shortLabel(option), style = MaterialTheme.typography.bodySmall)
        }
    }
    if (note != null) {
        Text(note, style = MaterialTheme.typography.bodySmall)
    }

    Spacer(Modifier.height(8.dp))

    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.spacedBy(16.dp),
    ) {
        LimitCard(
            title = "Mínimo",
            valueDeg = minValue,
            timestamp = minTimestamp,
            containerColor = minCardColor,
            modifier = Modifier.weight(1f),
        )
        LimitCard(
            title = "Máximo",
            valueDeg = maxValue,
            timestamp = maxTimestamp,
            containerColor = maxCardColor,
            modifier = Modifier.weight(1f),
        )
    }
}

@Composable
private fun LimitCard(
    title: String,
    valueDeg: Double?,
    timestamp: Long?,
    containerColor: Color,
    modifier: Modifier = Modifier,
) {
    Card(
        modifier = modifier,
        colors = CardDefaults.cardColors(containerColor = containerColor),
    ) {
        Column(
            modifier = Modifier.padding(12.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
        ) {
            Text(title, fontWeight = FontWeight.Bold)
            Text(
                valueDeg?.let { "%.2f°".format(it) } ?: "--.--°",
                style = MaterialTheme.typography.titleLarge,
            )
            Text(timestamp?.let { timeFormatter.format(Date(it)) } ?: "")
        }
    }
}

@Composable
private fun ModeSelector(
    mode: ConnectionMode,
    onModeChange: (ConnectionMode) -> Unit,
    tiltDisplayStepDeg: Double,
    onTiltDisplayStepChange: (Double) -> Unit,
    panDisplayStepDeg: Double,
    onPanDisplayStepChange: (Double) -> Unit,
    bleAddress: String,
    onBleAddressChange: (String) -> Unit,
    scanning: Boolean,
    scanResults: List<BleScanner.Found>,
    onScanClick: () -> Unit,
    onScanResultSelected: (String) -> Unit,
    onScanDialogDismiss: () -> Unit,
    testInProgress: Boolean,
    testResult: String?,
    testSuccess: Boolean,
    onTestConnectionClick: () -> Unit,
) {
    var showScanDialog by remember { mutableStateOf(false) }

    Column {
        Row(verticalAlignment = Alignment.CenterVertically) {
            RadioButton(selected = mode == ConnectionMode.SIMULATED, onClick = { onModeChange(ConnectionMode.SIMULATED) })
            Text("Simulação")
            Spacer(Modifier.width(16.dp))
            RadioButton(selected = mode == ConnectionMode.REAL, onClick = { onModeChange(ConnectionMode.REAL) })
            Text("Real (BLE)")
        }
        if (mode == ConnectionMode.REAL) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                OutlinedTextField(
                    value = bleAddress,
                    onValueChange = onBleAddressChange,
                    label = { Text("Endereço MAC do dispositivo BLE") },
                    modifier = Modifier.weight(1f),
                )
                Spacer(Modifier.width(8.dp))
                OutlinedButton(onClick = {
                    showScanDialog = true
                    onScanClick()
                }) {
                    Text("Escanear")
                }
            }
            Spacer(Modifier.height(8.dp))
            OutlinedButton(onClick = onTestConnectionClick, enabled = !testInProgress) {
                Text(if (testInProgress) "Testando..." else "Testar conexão com ESP32")
            }
            testResult?.let {
                Text(
                    it,
                    color = if (testSuccess) Green else Red,
                    fontWeight = FontWeight.Bold,
                    style = MaterialTheme.typography.bodySmall,
                )
            }
        }

        for ((label, current, onChange) in listOf(
            Triple("Resolução da inclinação:", tiltDisplayStepDeg, onTiltDisplayStepChange),
            Triple("Resolução do azimute:", panDisplayStepDeg, onPanDisplayStepChange),
        )) {
            Spacer(Modifier.height(8.dp))
            Text(label, style = MaterialTheme.typography.bodySmall)
            Row(verticalAlignment = Alignment.CenterVertically) {
                AngleDisplay.STEPS_DEG.forEachIndexed { i, option ->
                    if (i > 0) Spacer(Modifier.width(16.dp))
                    RadioButton(selected = current == option, onClick = { onChange(option) })
                    Text(AngleDisplay.label(option))
                }
            }
        }
    }

    if (showScanDialog) {
        BleScanDialog(
            scanning = scanning,
            results = scanResults,
            onSelect = { address ->
                onScanResultSelected(address)
                showScanDialog = false
            },
            onDismiss = {
                showScanDialog = false
                onScanDialogDismiss()
            },
        )
    }
}

@Composable
private fun BleScanDialog(
    scanning: Boolean,
    results: List<BleScanner.Found>,
    onSelect: (String) -> Unit,
    onDismiss: () -> Unit,
) {
    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Dispositivos BLE") },
        text = {
            Column {
                if (scanning) {
                    Text("Escaneando...", style = MaterialTheme.typography.bodySmall)
                    Spacer(Modifier.height(8.dp))
                    LinearProgressIndicator(modifier = Modifier.fillMaxWidth())
                    Spacer(Modifier.height(12.dp))
                }
                if (results.isEmpty()) {
                    if (!scanning) {
                        Text("Nenhum dispositivo BLE encontrado por perto.", style = MaterialTheme.typography.bodySmall)
                    }
                } else {
                    results.forEach { found ->
                        Text(
                            "${found.name} (${found.address})",
                            modifier = Modifier
                                .fillMaxWidth()
                                .clickable { onSelect(found.address) }
                                .padding(vertical = 8.dp),
                        )
                    }
                }
            }
        },
        confirmButton = { TextButton(onClick = onDismiss) { Text("Fechar") } },
    )
}

private fun sharePdf(context: Context, file: File) {
    try {
        val uri = FileProvider.getUriForFile(context, "${context.packageName}.fileprovider", file)
        val intent = Intent(Intent.ACTION_SEND).apply {
            type = "application/pdf"
            putExtra(Intent.EXTRA_STREAM, uri)
            addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
        }
        context.startActivity(Intent.createChooser(intent, "Compartilhar relatório"))
    } catch (e: Exception) {
        // O PDF já está salvo; só o compartilhamento falhou — sem derrubar o app.
        Toast.makeText(context, "Relatório salvo, mas não foi possível compartilhar: ${e.message}", Toast.LENGTH_LONG).show()
    }
}
