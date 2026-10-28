"""Diálogo de configurações: escolha de modo (simulado / USB / BLE) e
parâmetros de conexão de cada transporte."""
from __future__ import annotations

from dataclasses import dataclass

from PyQt5.QtCore import QSettings, pyqtSignal
from PyQt5.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
)

from ui.angle_display import DEFAULT_DISPLAY_STEP_DEG, DISPLAY_STEPS_DEG, normalize_step, step_label

try:
    from serial.tools import list_ports
except ImportError:  # pyserial pode não estar instalado em modo só-simulação
    list_ports = None


@dataclass
class AppSettings:
    mode: str = "simulado"  # "simulado" | "real" (USB/Modbus RTU) | "ble"
    serial_port: str = ""
    baudrate: int = 9600
    slave_id: int = 1
    ble_address: str = ""
    # Degrau de exibição de cada eixo na tela principal (ver
    # ui/angle_display.py). 0,25° é o padrão; 0,1° é para bancada, comparando
    # com uma referência de precisão (ex: nível eletrônico Mitutoyo / mesa
    # giratória). Cada eixo tem o seu — trocar um não afeta o outro. Também
    # trocável direto no cartão de cada eixo, e lembrado entre sessões
    # (load_display_steps/save_display_steps).
    tilt_display_step_deg: float = DEFAULT_DISPLAY_STEP_DEG
    pan_display_step_deg: float = DEFAULT_DISPLAY_STEP_DEG


# Onde a resolução de exibição fica guardada entre sessões: um arquivo .ini
# do usuário (no Windows, %APPDATA%\Avibras\Inclinometro.ini), via QSettings.
_SETTINGS_ORG = "Avibras"
_SETTINGS_APP = "Inclinometro"


def _store() -> QSettings:
    return QSettings(QSettings.IniFormat, QSettings.UserScope, _SETTINGS_ORG, _SETTINGS_APP)


def load_display_steps(settings: AppSettings) -> AppSettings:
    """Preenche os degraus de exibição gravados na última sessão."""
    store = _store()
    for key in ("tilt_display_step_deg", "pan_display_step_deg"):
        try:
            value = float(store.value(f"exibicao/{key}", getattr(settings, key)))
        except (TypeError, ValueError):
            continue
        setattr(settings, key, normalize_step(value))
    return settings


def save_display_steps(settings: AppSettings) -> None:
    store = _store()
    store.setValue("exibicao/tilt_display_step_deg", settings.tilt_display_step_deg)
    store.setValue("exibicao/pan_display_step_deg", settings.pan_display_step_deg)
    store.sync()


class SettingsDialog(QDialog):
    _devices_found = pyqtSignal(list, str)  # [(endereco, nome)], erro (vazio se ok)
    _port_detected = pyqtSignal(str, str)  # porta encontrada (vazio se não achou), erro
    _test_finished = pyqtSignal(bool, str)  # sucesso, texto do resultado

    def __init__(self, current: AppSettings, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Configurações")
        self._devices_found.connect(self._on_devices_found)
        self._port_detected.connect(self._on_port_detected)
        self._test_finished.connect(self._on_test_finished)

        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Simulação", "simulado")
        self.mode_combo.addItem("Real (USB/Modbus RTU)", "real")
        self.mode_combo.addItem("Real (Bluetooth BLE)", "ble")
        idx = self.mode_combo.findData(current.mode)
        if idx >= 0:
            self.mode_combo.setCurrentIndex(idx)
        self.mode_combo.currentIndexChanged.connect(self._update_visible_fields)

        # --- USB/Modbus RTU ---
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self._populate_ports()
        if current.serial_port:
            self.port_combo.setCurrentText(current.serial_port)

        self.detect_port_btn = QPushButton("Detectar automaticamente")
        self.detect_port_btn.clicked.connect(self._detect_port)

        port_row = QHBoxLayout()
        port_row.addWidget(self.port_combo, 1)
        port_row.addWidget(self.detect_port_btn)

        self.baud_combo = QComboBox()
        for baud in (9600, 19200, 38400, 57600, 115200):
            self.baud_combo.addItem(str(baud), baud)
        idx = self.baud_combo.findData(current.baudrate)
        if idx >= 0:
            self.baud_combo.setCurrentIndex(idx)

        self.slave_spin = QSpinBox()
        self.slave_spin.setRange(1, 247)
        self.slave_spin.setValue(current.slave_id)

        self.tilt_resolution_combo = QComboBox()
        self.pan_resolution_combo = QComboBox()
        for combo, step in ((self.tilt_resolution_combo, current.tilt_display_step_deg),
                            (self.pan_resolution_combo, current.pan_display_step_deg)):
            for option in DISPLAY_STEPS_DEG:
                combo.addItem(step_label(option), option)
            idx = combo.findData(normalize_step(step))
            if idx >= 0:
                combo.setCurrentIndex(idx)

        self.usb_port_label = QLabel("Porta serial:")
        self.usb_baud_label = QLabel("Baud rate:")
        self.usb_slave_label = QLabel("Endereço Modbus (slave id):")

        # --- Bluetooth BLE ---
        self.ble_combo = QComboBox()
        self.ble_combo.setEditable(True)
        if current.ble_address:
            self.ble_combo.setCurrentText(current.ble_address)

        self.scan_ble_btn = QPushButton("Escanear")
        self.scan_ble_btn.clicked.connect(self._scan_ble_devices)

        ble_row = QHBoxLayout()
        ble_row.addWidget(self.ble_combo, 1)
        ble_row.addWidget(self.scan_ble_btn)
        self.ble_row_label = QLabel("Dispositivo BLE:")

        form = QFormLayout()
        form.addRow("Modo:", self.mode_combo)
        form.addRow(self.usb_port_label, port_row)
        form.addRow(self.usb_baud_label, self.baud_combo)
        form.addRow(self.usb_slave_label, self.slave_spin)
        form.addRow(self.ble_row_label, ble_row)
        form.addRow("Resolução da inclinação:", self.tilt_resolution_combo)
        form.addRow("Resolução do azimute:", self.pan_resolution_combo)

        self.test_btn = QPushButton("Testar conexão com ESP32")
        self.test_btn.clicked.connect(self._test_connection)
        self.test_result_label = QLabel("")

        test_row = QHBoxLayout()
        test_row.addWidget(self.test_btn)
        test_row.addWidget(self.test_result_label, 1)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)

        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addLayout(test_row)
        layout.addWidget(buttons)

        self._update_visible_fields()

    def _update_visible_fields(self) -> None:
        mode = self.mode_combo.currentData()
        for widget in (self.usb_port_label, self.port_combo, self.detect_port_btn,
                       self.usb_baud_label, self.baud_combo, self.usb_slave_label, self.slave_spin):
            widget.setVisible(mode == "real")
        self.ble_row_label.setVisible(mode == "ble")
        self.ble_combo.setVisible(mode == "ble")
        self.scan_ble_btn.setVisible(mode == "ble")
        self.test_btn.setVisible(mode in ("real", "ble"))
        self.test_result_label.clear()

    def _test_connection(self) -> None:
        """Roda o teste numa thread: ele pode levar vários segundos (a placa
        pode reiniciar ao abrir a porta, e sem resposta o teste ainda reabre
        a porta uma vez), e na thread da UI a janela congelava enquanto isso."""
        import threading

        mode = self.mode_combo.currentData()
        if mode not in ("real", "ble"):
            return
        port = self.port_combo.currentText().strip()
        baudrate = self.baud_combo.currentData()
        slave_id = self.slave_spin.value()
        # currentData() guarda o endereço puro (ex: "00:70:07:25:60:8A") quando
        # o item veio do "Escanear"; currentText() nesse caso é o rótulo
        # exibido "Nome (endereço)" inteiro, que o bleak não reconhece como
        # endereço válido — daí cair para currentText() só quando não há
        # currentData (usuário digitou o endereço à mão).
        address = self.ble_combo.currentData() or self.ble_combo.currentText().strip()

        self.test_btn.setEnabled(False)
        self.test_result_label.setText("Testando... (pode levar alguns segundos)")
        self.test_result_label.setStyleSheet("color: #888;")

        def worker() -> None:
            try:
                if mode == "real":
                    if not port:
                        raise ValueError("Selecione uma porta serial.")
                    from data_source.modbus_source import test_connection

                    result = test_connection(port, baudrate, slave_id)
                else:
                    if not address:
                        raise ValueError("Selecione ou informe um endereço BLE.")
                    from data_source.ble_source import test_connection

                    result = test_connection(address)
                pan_text = (
                    f", azimute {result.pan_deg:.2f}°"
                    if result.pan_deg is not None
                    else ", sem eixo de azimute"
                )
                ok, text = True, (
                    f"✓ ESP32 respondeu — inclinação {result.angle_deg:.2f}°{pan_text} "
                    f"(firmware v{result.firmware_version})"
                )
            except Exception as exc:  # noqa: BLE001
                ok, text = False, f"✗ Falha: {exc}"
            try:
                self._test_finished.emit(ok, text)
            except RuntimeError:  # diálogo já fechado
                pass

        threading.Thread(target=worker, daemon=True).start()

    def _on_test_finished(self, ok: bool, text: str) -> None:
        self.test_btn.setEnabled(True)
        self.test_result_label.setText(text)
        self.test_result_label.setStyleSheet(
            "color: #2e7d32; font-weight: bold;" if ok else "color: #c62828; font-weight: bold;"
        )

    def _detect_port(self) -> None:
        import threading

        baudrate = self.baud_combo.currentData()
        slave_id = self.slave_spin.value()

        self.detect_port_btn.setEnabled(False)
        self.test_result_label.setText("Detectando porta do ESP32...")
        self.test_result_label.setStyleSheet("color: #888;")

        def worker() -> None:
            try:
                from data_source.modbus_source import find_port

                port = find_port(baudrate, slave_id)
                self._port_detected.emit(port or "", "")
            except Exception as exc:  # noqa: BLE001
                self._port_detected.emit("", str(exc))

        threading.Thread(target=worker, daemon=True).start()

    def _on_port_detected(self, port: str, error: str) -> None:
        self.detect_port_btn.setEnabled(True)
        if error:
            self.test_result_label.setText(f"✗ Falha ao detectar: {error}")
            self.test_result_label.setStyleSheet("color: #c62828; font-weight: bold;")
            return
        if not port:
            self.test_result_label.setText(
                "Nenhum ESP32 respondeu em nenhuma porta serial (confira o cabo USB e o baud rate)."
            )
            self.test_result_label.setStyleSheet("color: #c62828; font-weight: bold;")
            return
        self.port_combo.setCurrentText(port)
        self.test_result_label.setText(f"✓ ESP32 encontrado em {port}.")
        self.test_result_label.setStyleSheet("color: #2e7d32; font-weight: bold;")

    def _scan_ble_devices(self) -> None:
        import threading

        self.scan_ble_btn.setEnabled(False)
        self.test_result_label.setText("Escaneando dispositivos BLE...")
        self.test_result_label.setStyleSheet("color: #888;")

        def worker() -> None:
            try:
                from data_source.ble_source import scan_devices

                devices = scan_devices()
                self._devices_found.emit(devices, "")
            except Exception as exc:  # noqa: BLE001
                self._devices_found.emit([], str(exc))

        threading.Thread(target=worker, daemon=True).start()

    def _on_devices_found(self, devices: list, error: str) -> None:
        self.scan_ble_btn.setEnabled(True)
        if error:
            self.test_result_label.setText(f"✗ Falha ao escanear: {error}")
            self.test_result_label.setStyleSheet("color: #c62828; font-weight: bold;")
            return
        if not devices:
            self.test_result_label.setText("Nenhum dispositivo BLE encontrado por perto.")
            self.test_result_label.setStyleSheet("color: #888;")
            return
        current_text = self.ble_combo.currentText()
        self.ble_combo.clear()
        for address, name in devices:
            self.ble_combo.addItem(f"{name} ({address})", address)
        if current_text:
            self.ble_combo.setCurrentText(current_text)
        self.test_result_label.setText(f"{len(devices)} dispositivo(s) encontrado(s).")
        self.test_result_label.setStyleSheet("color: #2e7d32; font-weight: bold;")

    def _populate_ports(self) -> None:
        if list_ports is None:
            return
        for p in list_ports.comports():
            self.port_combo.addItem(p.device)

    def result_settings(self) -> AppSettings:
        ble_address = self.ble_combo.currentData() or self.ble_combo.currentText().strip()
        return AppSettings(
            mode=self.mode_combo.currentData(),
            serial_port=self.port_combo.currentText().strip(),
            baudrate=self.baud_combo.currentData(),
            slave_id=self.slave_spin.value(),
            ble_address=ble_address,
            tilt_display_step_deg=self.tilt_resolution_combo.currentData(),
            pan_display_step_deg=self.pan_resolution_combo.currentData(),
        )
