"""Janela principal: exibição do ângulo em tempo real, destaque de novos
limites (mín/máx), e ações de reset/relatório."""
from __future__ import annotations

import datetime as _dt
import os
import threading

from PyQt5.QtCore import QObject, QStandardPaths, Qt, QTimer, QUrl, pyqtSignal
from PyQt5.QtGui import QDesktopServices, QFont, QPixmap
from PyQt5.QtWidgets import (
    QComboBox,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QProgressDialog,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from app_version import APP_NAME, APP_VERSION
from data_source.base import MIN_RECOMMENDED_FIRMWARE, AngleReading, IAngleDataSource, firmware_outdated
from data_source.ble_source import BleAngleSource
from data_source.modbus_source import ModbusAngleSource
from data_source.simulated_source import SimulatedAngleSource
from limits.history_store import HistoryStore
from limits.limit_tracker import PAN_AXIS, TILT_AXIS, LimitTracker
from limits.vibration_stats import analyze_axis, has_pan_samples
from report.report_generator import generate_report, generate_vibration_report
from ui.angle_display import DISPLAY_STEPS_DEG, angle_for_display, format_angle, normalize_step, short_step_label
from ui.settings_dialog import AppSettings, SettingsDialog, load_display_steps, save_display_steps
from ui.vibration_dialog import VibrationConfigDialog, VibrationResultDialog

# Paleta Avibras Aeroco (fundo azul marinho + detalhes laranja). O logo é
# opcional: se `assets/logo.<ext>` existir (png, jpg ou jpeg), é exibido no
# canto superior direito; caso contrário, o título em texto já usa a mesma
# identidade visual.
NAVY = "#0B2145"
NAVY_PANEL = "#15305F"  # tom mais claro que o fundo, para destacar painéis/caixas
ORANGE = "#F5821F"
TEXT_LIGHT = "#F4F6F9"
GREEN = "#2e7d32"
RED = "#c62828"
_ASSETS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "assets")


def _find_logo_path() -> str | None:
    for ext in ("png", "jpg", "jpeg"):
        candidate = os.path.join(_ASSETS_DIR, f"logo.{ext}")
        if os.path.isfile(candidate):
            return candidate
    return None


LOGO_PATH = _find_logo_path()

# Exibição da leitura contínua: degrau por eixo (0,25° padrão ou 0,1°), com
# histerese — a regra mora em ui/angle_display.py, a mesma do app Android.
# Nada disso afeta os dados guardados: histórico, mín/máx e relatório usam
# sempre o ângulo bruto.

_VALUE_STYLE = f"font-size: 24px; font-weight: bold; color: {TEXT_LIGHT}; padding: 4px;"
_FLASH_STYLE = f"font-size: 24px; font-weight: bold; background-color: {ORANGE}; color: {NAVY}; border-radius: 4px; padding: 4px;"

_BADGE_STYLE = "font-size: 15px; font-weight: bold; border-radius: 10px; padding: 6px 16px;"

_CONN_STYLES = {
    "parado": ("○ Parado", f"color: #9aa5b1; background: transparent;"),
    "conectando": ("◐ Conectando...", f"color: {ORANGE}; background: transparent; font-weight: bold;"),
    "conectado": ("● Conectado", f"{_BADGE_STYLE} color: white; background-color: {GREEN};"),
    "erro": ("● Falha de conexão", f"{_BADGE_STYLE} color: white; background-color: {RED};"),
    "simulacao": ("● Simulação (interna)", f"color: {ORANGE}; background: transparent; font-weight: bold;"),
}

_APP_STYLESHEET = f"""
QMainWindow, QWidget {{
    background-color: {NAVY};
    color: {TEXT_LIGHT};
}}
QLabel {{
    color: {TEXT_LIGHT};
}}
QPushButton {{
    background-color: {ORANGE};
    color: {NAVY};
    border: none;
    border-radius: 6px;
    padding: 12px 18px;
    font-weight: bold;
    font-size: 14px;
}}
QPushButton:hover {{
    background-color: #ff9d40;
}}
QPushButton:pressed {{
    background-color: #cf6a12;
}}
QPushButton:disabled {{
    background-color: #5a6472;
    color: #cfd4da;
}}
QFrame {{
    background-color: {NAVY_PANEL};
    border: 2px solid {ORANGE};
    border-radius: 6px;
}}
QStatusBar {{
    color: {TEXT_LIGHT};
}}
"""


class _SignalBridge(QObject):
    """Ponte thread-safe: os data sources chamam callbacks em threads próprias;
    aqui os dados viram sinais Qt (fila automática para a thread da UI)."""

    reading = pyqtSignal(object)  # AngleReading
    error = pyqtSignal(str)
    calibration_done = pyqtSignal(bool, str)
    vibration_progress = pyqtSignal(float, str)  # percentual, rótulo da fase
    vibration_done = pyqtSignal(object, str, float, float)  # amostras|None, erro, duration_s, rate_hz


def _fmt_time(ts: float) -> str:
    return _dt.datetime.fromtimestamp(ts).strftime("%H:%M:%S")


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(f"{APP_NAME} — Painel Desktop")
        # A janela abre maximizada (ver main.py, `showMaximized()`); este
        # tamanho só vale para quando o usuário desmaximiza manualmente, daí
        # ser bem maior que o mínimo abaixo.
        self.resize(1280, 800)
        self.setMinimumSize(1000, 640)
        self.setStyleSheet(_APP_STYLESHEET)

        # A resolução de exibição de cada eixo é lembrada entre sessões.
        self._settings = load_display_steps(AppSettings())
        self._history = HistoryStore()
        # Um rastreador de mín/máx por eixo: os extremos de inclinação e de
        # azimute são independentes.
        self._trackers = {TILT_AXIS: LimitTracker(TILT_AXIS), PAN_AXIS: LimitTracker(PAN_AXIS)}
        self._source: IAngleDataSource | None = None
        self._running = False
        self._vibration_progress_dialog: QProgressDialog | None = None
        self._firmware_warned = False
        # Pasta do último relatório salvo (ver `_ask_report_path`).
        self._report_dir: str | None = None
        # Degrau atualmente exibido em cada eixo (histerese da exibição) e a
        # última leitura bruta, para redesenhar na hora ao trocar a resolução.
        self._displayed: dict[str, float | None] = {TILT_AXIS: None, PAN_AXIS: None}
        self._last_value: dict[str, float | None] = {TILT_AXIS: None, PAN_AXIS: None}

        self._bridge = _SignalBridge()
        self._bridge.reading.connect(self._on_reading)
        self._bridge.error.connect(self._on_error)
        self._bridge.calibration_done.connect(self._on_calibration_done)
        self._bridge.vibration_progress.connect(self._on_vibration_progress)
        self._bridge.vibration_done.connect(self._on_vibration_done)

        self._build_ui()
        self._update_mode_label()
        self._set_connection_status("parado")
        self._update_version_label()

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        # Margens generosas e espaçamento maior: numa janela maximizada, sem
        # isso o conteúdo fica colado nas bordas e amontoado no topo,
        # deixando um vão vazio embaixo — a origem da impressão de
        # "desproporcional" relatada.
        root.setContentsMargins(28, 20, 28, 20)
        root.setSpacing(18)

        root.addLayout(self._build_header())

        status_row = QHBoxLayout()
        status_row.setSpacing(24)
        self.mode_label = QLabel()
        self.mode_label.setAlignment(Qt.AlignCenter)
        self.mode_label.setStyleSheet("font-size: 14px;")
        self.connection_label = QLabel()
        self.connection_label.setAlignment(Qt.AlignCenter)
        # Versões do app e do firmware conectado (ver `_update_version_label`).
        self.version_label = QLabel()
        self.version_label.setAlignment(Qt.AlignCenter)
        status_row.addStretch(1)
        status_row.addWidget(self.mode_label)
        status_row.addWidget(self.connection_label)
        status_row.addWidget(self.version_label)
        status_row.addStretch(1)
        root.addLayout(status_row)

        # Os dois eixos lado a lado, cada um em um cartão do mesmo tamanho
        # (stretch 1 para os dois), com peso 1 na coluna principal para
        # ocupar o espaço vertical que sobra numa janela maximizada.
        axes_row = QHBoxLayout()
        axes_row.setSpacing(24)
        self._axis_widgets = {
            TILT_AXIS: self._build_axis_panel("Inclinação (tilt)", TILT_AXIS),
            PAN_AXIS: self._build_axis_panel("Azimute (pan)", PAN_AXIS),
        }
        axes_row.addWidget(self._axis_widgets[TILT_AXIS]["frame"], 1)
        axes_row.addWidget(self._axis_widgets[PAN_AXIS]["frame"], 1)
        root.addLayout(axes_row, 1)

        buttons_row = QHBoxLayout()
        buttons_row.setSpacing(16)
        self.start_stop_btn = QPushButton("Iniciar")
        self.start_stop_btn.clicked.connect(self._toggle_start_stop)
        self.reset_btn = QPushButton("Resetar limites")
        self.reset_btn.clicked.connect(self._reset_limits)
        self.calibrate_btn = QPushButton("Calibrar")
        self.calibrate_btn.clicked.connect(self._calibrate)
        self.vibration_btn = QPushButton("Modo Vibração")
        self.vibration_btn.clicked.connect(self._start_vibration_capture)
        self.settings_btn = QPushButton("Configurações...")
        self.settings_btn.clicked.connect(self._open_settings)
        self.report_btn = QPushButton("Gerar relatório PDF")
        self.report_btn.clicked.connect(self._generate_report)
        # Peso igual (stretch 1) para os seis: preenchem a largura toda de
        # forma uniforme em vez de ficarem amontoados de um lado só, com o
        # resto da barra vazio, como numa janela maximizada larga.
        for btn in (
            self.start_stop_btn,
            self.reset_btn,
            self.calibrate_btn,
            self.vibration_btn,
            self.settings_btn,
            self.report_btn,
        ):
            btn.setMinimumHeight(46)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            buttons_row.addWidget(btn, 1)
        root.addLayout(buttons_row)

        self.statusBar().showMessage("Pronto.")

    def _build_header(self) -> QHBoxLayout:
        header = QHBoxLayout()

        title_label = QLabel(APP_NAME)
        title_label.setStyleSheet(f"font-size: 22px; font-weight: bold; color: {TEXT_LIGHT};")
        header.addWidget(title_label)

        header.addStretch(1)

        if LOGO_PATH is not None:
            logo_label = QLabel()
            pixmap = QPixmap(LOGO_PATH)
            if not pixmap.isNull():
                logo_label.setPixmap(pixmap.scaledToHeight(52, Qt.SmoothTransformation))
                # A logo tem fundo branco (não transparente) — um "cartão"
                # branco arredondado ao redor evita que pareça um retângulo
                # solto sobre o fundo azul marinho do cabeçalho.
                logo_label.setStyleSheet("background-color: white; border-radius: 6px; padding: 6px 10px;")
                header.addWidget(logo_label, 0, Qt.AlignRight)
        else:
            logo_label = QLabel("AVIBRAS aeroco")
            logo_label.setStyleSheet(f"font-size: 16px; font-weight: bold; color: {ORANGE};")
            header.addWidget(logo_label, 0, Qt.AlignRight)

        return header

    def _build_axis_panel(self, title: str, axis: str) -> dict:
        """Cartão de um eixo: título, valor grande, seletor de resolução,
        aviso opcional e o par de caixas de mínimo/máximo. Devolve os widgets num dicionário, para o
        resto da janela atualizar os dois eixos pelo mesmo caminho de código.

        É um QFrame (e não só um layout) de propósito: os dois eixos ficam
        como dois cartões do mesmo tamanho, com a mesma borda usada nas
        caixas de mínimo/máximo — o mesmo padrão visual, só numa escala
        maior — em vez de dois blocos de texto soltos sobre o fundo."""
        frame = QFrame()
        frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        frame.setMinimumHeight(320)
        column = QVBoxLayout(frame)
        column.setContentsMargins(20, 18, 20, 18)
        column.setSpacing(10)

        title_label = QLabel(title)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet(f"font-size: 17px; font-weight: bold; color: {ORANGE};")
        column.addWidget(title_label)

        # Empurra o valor para o centro vertical do cartão, em vez de deixar
        # tudo colado no topo enquanto o cartão cresce numa janela maximizada.
        column.addStretch(1)

        value_label = QLabel("--.--°")
        value_label.setAlignment(Qt.AlignCenter)
        value_label.setFont(QFont("Sans Serif", 56, QFont.Bold))
        column.addWidget(value_label)

        # Resolução da tela deste eixo, à mão no próprio cartão (a mesma opção
        # de Configurações): 0,25° padrão ou 0,1° para bancada.
        resolution_row = QHBoxLayout()
        resolution_row.addStretch(1)
        resolution_label = QLabel("Resolução:")
        resolution_label.setStyleSheet("font-size: 13px; color: #9aa5b1; border: none;")
        resolution_combo = QComboBox()
        for option in DISPLAY_STEPS_DEG:
            resolution_combo.addItem(short_step_label(option), option)
        idx = resolution_combo.findData(self._display_step(axis))
        if idx >= 0:
            resolution_combo.setCurrentIndex(idx)
        resolution_combo.currentIndexChanged.connect(
            lambda _i, a=axis, c=resolution_combo: self._set_display_step(a, c.currentData())
        )
        resolution_row.addWidget(resolution_label)
        resolution_row.addWidget(resolution_combo)
        resolution_row.addStretch(1)
        column.addLayout(resolution_row)

        # Normalmente oculto; usado para explicar por que um eixo está sem
        # valor (ex: firmware antigo, sem azimute). Fica escondido em vez de
        # só vazio porque a folha de estilo desta janela dá borda a QFrame, e
        # QLabel herda de QFrame — um rótulo vazio apareceria como uma faixa
        # com borda no meio do painel.
        note_label = QLabel("")
        note_label.setAlignment(Qt.AlignCenter)
        note_label.setStyleSheet("font-size: 12px; color: #9aa5b1;")
        note_label.setVisible(False)
        column.addWidget(note_label)

        column.addStretch(1)

        limits_row = QHBoxLayout()
        limits_row.setSpacing(14)
        min_frame, min_value_label, min_time_label = self._build_limit_box("Mínimo")
        max_frame, max_value_label, max_time_label = self._build_limit_box("Máximo")
        limits_row.addWidget(min_frame, 1)
        limits_row.addWidget(max_frame, 1)
        column.addLayout(limits_row)

        return {
            "frame": frame,
            "value": value_label,
            "resolution": resolution_combo,
            "note": note_label,
            "min_value": min_value_label,
            "min_time": min_time_label,
            "max_value": max_value_label,
            "max_time": max_time_label,
        }

    def _build_limit_box(self, title: str) -> tuple[QFrame, QLabel, QLabel]:
        frame = QFrame()
        frame.setFrameShape(QFrame.StyledPanel)
        frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        frame.setMinimumHeight(100)
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(4)

        title_label = QLabel(title)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("font-size: 13px; font-weight: bold;")

        value_label = QLabel("--.--°")
        value_label.setAlignment(Qt.AlignCenter)
        value_label.setStyleSheet(_VALUE_STYLE)

        time_label = QLabel("")
        time_label.setAlignment(Qt.AlignCenter)
        time_label.setStyleSheet("font-size: 12px; color: #b8c0cb;")

        layout.addWidget(title_label)
        layout.addWidget(value_label)
        layout.addWidget(time_label)
        return frame, value_label, time_label

    def _update_mode_label(self) -> None:
        modo = {
            "simulado": "Simulação",
            "real": "Real (USB/Modbus RTU)",
            "ble": "Real (Bluetooth BLE)",
        }[self._settings.mode]
        estado = "em execução" if self._running else "parado"
        self.mode_label.setText(f"Modo: {modo} — {estado}")

    def _update_version_label(self) -> None:
        """Mostra "App vX · Firmware vY". A versão do firmware é lida pela
        fonte ao conectar; firmware mais antigo que o recomendado aparece em
        âmbar, com aviso na barra de status uma vez por sessão."""
        app = f"App v{APP_VERSION}"
        style = "font-size: 13px; color: #9aa7b5;"
        if self._source is None:
            firmware = "Firmware: —"
        elif self._settings.mode == "simulado":
            firmware = "Firmware: simulação"
        else:
            version = self._source.firmware_version
            firmware = f"Firmware v{version}" if version else "Firmware: lendo..."
            if firmware_outdated(version):
                recommended = ".".join(str(p) for p in MIN_RECOMMENDED_FIRMWARE)
                firmware += f" — desatualizado, grave a {recommended}"
                style = "font-size: 13px; color: #f0a020; font-weight: bold;"
                if not self._firmware_warned:
                    self._firmware_warned = True
                    self.statusBar().showMessage(
                        f"Firmware v{version} desatualizado: grave a versão {recommended} no ESP32 "
                        f"(correções de leitura do pan e do Modo Vibração)."
                    )
        self.version_label.setText(f"{app}  ·  {firmware}")
        self.version_label.setStyleSheet(style)

    def _set_connection_status(self, status: str) -> None:
        text, style = _CONN_STYLES[status]
        self.connection_label.setText(text)
        self.connection_label.setStyleSheet(style)

    # --------------------------------------------------------------- ações
    def _open_settings(self) -> None:
        # Para a conexão ANTES de abrir o diálogo (não só depois de aceitar):
        # senão, com uma conexão Real já em andamento (mesmo que falhando nas
        # leituras), a porta serial continua aberta pela thread de fundo
        # enquanto o diálogo está na tela, e "Detectar automaticamente"/
        # "Testar conexão" tentam abrir essa mesma porta já ocupada pelo
        # próprio app — reportando "nenhum ESP32 encontrado" mesmo com o
        # hardware são. Reinicia ao fechar o diálogo (aceito ou cancelado)
        # para não deixar o usuário precisando clicar em "Iniciar" de novo.
        was_running = self._running
        if was_running:
            self._stop()
        dialog = SettingsDialog(self._settings, self)
        if dialog.exec_() == SettingsDialog.Accepted:
            self._settings = dialog.result_settings()
            self._update_mode_label()
            save_display_steps(self._settings)
            for axis in (TILT_AXIS, PAN_AXIS):
                self._sync_display_step(axis)
        if was_running:
            self._start()

    def _toggle_start_stop(self) -> None:
        if self._running:
            self._stop()
        else:
            self._start()

    def _start(self) -> None:
        if self._settings.mode == "real" and not self._settings.serial_port:
            QMessageBox.warning(self, "Configuração incompleta", "Selecione uma porta serial nas Configurações.")
            return
        if self._settings.mode == "ble" and not self._settings.ble_address:
            QMessageBox.warning(self, "Configuração incompleta", "Selecione ou informe um dispositivo BLE nas Configurações.")
            return

        if self._settings.mode == "simulado":
            self._source = SimulatedAngleSource()
        elif self._settings.mode == "ble":
            self._source = BleAngleSource(device_address=self._settings.ble_address)
        else:
            self._source = ModbusAngleSource(
                port=self._settings.serial_port,
                baudrate=self._settings.baudrate,
                slave_id=self._settings.slave_id,
            )

        for tracker in self._trackers.values():
            tracker.reset()
        self._displayed = {TILT_AXIS: None, PAN_AXIS: None}
        self._history.start_session(mode=self._settings.mode)
        self._source.start(
            on_reading=lambda r: self._bridge.reading.emit(r),
            on_error=lambda msg: self._bridge.error.emit(msg),
        )
        self._running = True
        self.start_stop_btn.setText("Parar")
        self._update_mode_label()
        self._set_connection_status("simulacao" if self._settings.mode == "simulado" else "conectando")
        self.statusBar().showMessage(f"Conectado: {self._source.label}")
        self._firmware_warned = False
        self._update_version_label()

    def _stop(self) -> None:
        if self._source is not None:
            self._source.stop()
            self._source = None
        self._history.end_session()
        self._running = False
        self._displayed = {TILT_AXIS: None, PAN_AXIS: None}
        self.start_stop_btn.setText("Iniciar")
        self._update_mode_label()
        self._set_connection_status("parado")
        self.statusBar().showMessage("Parado.")
        self._update_version_label()

    def _reset_limits(self) -> None:
        was_running = self._running
        mode = self._settings.mode
        if was_running:
            self._history.end_session()
            self._history.start_session(mode=mode)

        # Com firmware v1.5.0+ quem guarda os extremos é o ESP32 (ver
        # `limits.limit_tracker`). Zerar só o lado do app não adiantaria: a
        # próxima leitura traria os extremos antigos de volta.
        #
        # A ordem importa e é esta: dispositivo primeiro, rastreadores depois.
        # Invertida, uma leitura chegando no meio reintroduziria os extremos
        # antigos — e, como mín/máx só andam para fora, eles ficariam. É por
        # isso também que esta chamada é síncrona aqui na thread da UI, e não
        # num worker: as leituras chegam por signal do Qt, ou seja, nesta mesma
        # thread, então nada consegue se intercalar entre as duas etapas.
        if self._source is not None and self._source.supports_peak_reset:
            try:
                self._source.reset_peaks()
            except Exception as exc:  # noqa: BLE001 - reset local segue de qualquer forma
                self.statusBar().showMessage(f"Falha ao resetar os extremos no dispositivo: {exc}")

        for axis, tracker in self._trackers.items():
            tracker.reset()
            widgets = self._axis_widgets[axis]
            widgets["min_value"].setText("--.--°")
            widgets["min_time"].setText("")
            widgets["max_value"].setText("--.--°")
            widgets["max_time"].setText("")
        self.statusBar().showMessage(
            "Limites resetados (histórico anterior preservado)." if was_running
            else "Limites resetados."
        )

    def _generate_report(self) -> None:
        session_id = self._history.current_session_id
        if session_id is None:
            sessions = self._history.list_sessions()
            if not sessions:
                QMessageBox.information(self, "Sem dados", "Ainda não há nenhuma sessão registrada.")
                return
            session_id = sessions[0].id

        path = self._ask_report_path("Salvar relatório", "relatorio_inclinometro")
        if not path:
            return

        readings = self._history.get_readings(session_id)
        events = self._history.get_limit_events(session_id)
        sessions_by_id = {s.id: s for s in self._history.list_sessions()}
        session_info = sessions_by_id.get(session_id)

        try:
            generate_report(path, session_info, readings, events)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "Erro ao gerar relatório", str(exc))
            return

        self._report_saved(path)

    def _ask_report_path(self, title: str, prefix: str) -> str:
        """Pergunta onde salvar um relatório, sugerindo um nome novo.

        A pasta sugerida era `os.getcwd()` — no app instalado, a pasta do
        programa, onde o usuário não pode gravar e o Windows redireciona para
        outro lugar. Os dois relatórios (sessão e vibração) acabavam num
        lugar que o usuário não escolheu, e ele abria de lá o arquivo errado
        (o de vibração salvo antes). Agora a sugestão é a pasta do último
        relatório salvo, começando por Documentos.
        """
        folder = self._report_dir or QStandardPaths.writableLocation(QStandardPaths.DocumentsLocation)
        default_name = f"{prefix}_{_dt.datetime.now():%Y%m%d_%H%M%S}.pdf"
        path, _ = QFileDialog.getSaveFileName(self, title, os.path.join(folder, default_name), "PDF (*.pdf)")
        if path and not path.lower().endswith(".pdf"):
            path += ".pdf"
        return path

    def _report_saved(self, path: str) -> None:
        """Confirma o relatório salvo e oferece abrir EXATAMENTE esse arquivo."""
        self._report_dir = os.path.dirname(path)
        answer = QMessageBox.question(
            self,
            "Relatório gerado",
            f"Relatório salvo em:\n{path}\n\nAbrir o relatório agora?",
            QMessageBox.Open | QMessageBox.Close,
            QMessageBox.Open,
        )
        if answer == QMessageBox.Open:
            QDesktopServices.openUrl(QUrl.fromLocalFile(path))

    def _display_step(self, axis: str) -> float:
        return self._settings.tilt_display_step_deg if axis == TILT_AXIS else self._settings.pan_display_step_deg

    def _set_display_step(self, axis: str, step_deg: float) -> None:
        """Troca a resolução da tela de um eixo (seletor do cartão ou
        Configurações), grava para a próxima sessão e redesenha na hora."""
        step_deg = normalize_step(step_deg)
        if axis == TILT_AXIS:
            self._settings.tilt_display_step_deg = step_deg
        else:
            self._settings.pan_display_step_deg = step_deg
        save_display_steps(self._settings)
        self._sync_display_step(axis)

    def _sync_display_step(self, axis: str) -> None:
        widgets = self._axis_widgets[axis]
        combo = widgets["resolution"]
        idx = combo.findData(self._display_step(axis))
        if idx >= 0 and idx != combo.currentIndex():
            combo.blockSignals(True)
            combo.setCurrentIndex(idx)
            combo.blockSignals(False)
        # O valor exibido estava no degrau antigo: recomeça a histerese.
        self._displayed[axis] = None
        if self._last_value[axis] is not None:
            widgets["value"].setText(self._display_text(axis, self._last_value[axis]))

    def _angle_for_display(self, axis: str, angle_deg: float) -> float:
        """Arredonda para o degrau de exibição do eixo, com histerese (ver
        ui/angle_display.py). Cada eixo tem seu próprio estado."""
        self._displayed[axis] = angle_for_display(angle_deg, self._displayed[axis], self._display_step(axis))
        return self._displayed[axis]

    def _display_text(self, axis: str, angle_deg: float) -> str:
        return format_angle(self._angle_for_display(axis, angle_deg), self._display_step(axis))

    # ------------------------------------------------------------ callbacks
    def _on_reading(self, reading: AngleReading) -> None:
        if self._settings.mode in ("real", "ble"):
            self._set_connection_status("conectado")
            if self._source is not None and not self.version_label.text().startswith(
                f"App v{APP_VERSION}  ·  Firmware v"
            ):
                self._update_version_label()
        self._history.add_reading(reading)

        values = {TILT_AXIS: reading.angle_deg, PAN_AXIS: reading.pan_deg}
        for axis, value in values.items():
            widgets = self._axis_widgets[axis]
            if value is None:
                # Firmware anterior à v1.2.0 não mede azimute — deixa claro
                # que o eixo está sem dado, em vez de mostrar um zero falso.
                widgets["value"].setText("--.--°")
                widgets["note"].setText("firmware sem este eixo")
                widgets["note"].setVisible(True)
                continue
            self._last_value[axis] = value
            widgets["value"].setText(self._display_text(axis, value))
            widgets["note"].setVisible(False)

            for event in self._trackers[axis].process(reading):
                self._history.add_limit_event(event)
                prefix = "min" if event.kind == "min" else "max"
                widgets[f"{prefix}_value"].setText(f"{event.value_deg:.2f}°")
                widgets[f"{prefix}_time"].setText(_fmt_time(event.reading.timestamp))
                self._flash(widgets[f"{prefix}_value"])

    def _on_error(self, message: str) -> None:
        if self._settings.mode in ("real", "ble"):
            self._set_connection_status("erro")
        # Sem timeout de propósito: com um timeout, a mensagem de erro some
        # sozinha depois de alguns segundos e a barra volta a mostrar o
        # "Conectado: ..." permanente deixado por `_start()` — dando a
        # impressão enganosa de que o problema se resolveu sozinho, quando
        # na real o badge "Falha de conexão" continua vermelho.
        self.statusBar().showMessage(message)

    def _calibrate(self) -> None:
        if self._source is None or not self._running:
            QMessageBox.warning(self, "Não conectado", "Inicie a leitura antes de calibrar.")
            return
        if not getattr(self._source, "supports_calibration", False):
            QMessageBox.information(self, "Calibração", "Esta fonte de dados não suporta calibração.")
            return

        source = self._source
        self.calibrate_btn.setEnabled(False)
        self.statusBar().showMessage("Calibrando...")

        def worker() -> None:
            try:
                source.calibrate()
                self._bridge.calibration_done.emit(True, "Calibração concluída: posição atual definida como 0°.")
            except Exception as exc:  # noqa: BLE001
                self._bridge.calibration_done.emit(False, f"Falha na calibração: {exc}")

        threading.Thread(target=worker, daemon=True).start()

    def _on_calibration_done(self, ok: bool, message: str) -> None:
        self.calibrate_btn.setEnabled(True)
        self.statusBar().showMessage(message, 5000)
        if ok:
            self._reset_limits()
        else:
            QMessageBox.warning(self, "Calibração", message)

    def _start_vibration_capture(self) -> None:
        if self._source is None or not self._running:
            QMessageBox.warning(self, "Não conectado", "Inicie a leitura antes de usar o Modo Vibração.")
            return
        if not getattr(self._source, "supports_vibration_capture", False):
            QMessageBox.information(self, "Modo Vibração", "Esta fonte de dados não suporta captura de vibração.")
            return

        dialog = VibrationConfigDialog(self)
        if dialog.exec_() != VibrationConfigDialog.Accepted:
            return
        duration_s, rate_hz = dialog.result_values()

        progress = QProgressDialog("Capturando...", "Cancelar", 0, 100, self)
        progress.setWindowTitle("Modo Vibração")
        progress.setWindowModality(Qt.WindowModal)
        progress.setMinimumDuration(0)
        progress.setValue(0)

        source = self._source
        progress.canceled.connect(lambda: getattr(source, "stop_vibration_capture", lambda: None)())
        self._vibration_progress_dialog = progress

        self.vibration_btn.setEnabled(False)
        self.statusBar().showMessage("Captura de vibração em andamento...")

        def on_progress(percent: float, phase: str = "") -> None:
            self._bridge.vibration_progress.emit(percent, phase)

        def on_done(readings: list[AngleReading] | None, error: str | None) -> None:
            self._bridge.vibration_done.emit(readings if readings is not None else [], error or "", duration_s, rate_hz)

        source.start_vibration_capture(duration_s, rate_hz, on_progress, on_done)

    def _on_vibration_progress(self, percent: float, phase: str = "") -> None:
        if self._vibration_progress_dialog is not None:
            if phase:
                self._vibration_progress_dialog.setLabelText(phase)
            self._vibration_progress_dialog.setValue(int(percent))

    def _on_vibration_done(self, readings: list, error: str, duration_s: float, rate_hz: float) -> None:
        self.vibration_btn.setEnabled(True)
        if self._vibration_progress_dialog is not None:
            self._vibration_progress_dialog.close()
            self._vibration_progress_dialog = None

        if error:
            self.statusBar().showMessage(f"Captura de vibração: {error}", 5000)
            if error != "Captura cancelada pelo usuário.":
                QMessageBox.warning(self, "Modo Vibração", error)
            return

        if not readings:
            QMessageBox.warning(self, "Modo Vibração", "Nenhuma amostra foi capturada.")
            return

        capture_id = self._history.save_vibration_capture(self._settings.mode, duration_s, rate_hz, readings)
        tilt = analyze_axis(readings, rate_hz, TILT_AXIS)
        # O eixo de azimute só existe na captura com firmware >= 1.3.0.
        pan = analyze_axis(readings, rate_hz, PAN_AXIS) if has_pan_samples(readings) else None
        self.statusBar().showMessage("Captura de vibração concluída.", 5000)

        result_dialog = VibrationResultDialog(tilt.stats, pan.stats if pan else None, self)
        result_dialog.exec_()
        if not result_dialog.save_requested:
            return

        path = self._ask_report_path("Salvar relatório de vibração", "relatorio_vibracao")
        if not path:
            return

        captures_by_id = {c.id: c for c in self._history.list_vibration_captures()}
        capture_info = captures_by_id.get(capture_id)
        try:
            generate_vibration_report(path, capture_info, readings, tilt, pan)
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "Erro ao gerar relatório", str(exc))
            return

        self._report_saved(path)

    def _flash(self, label: QLabel) -> None:
        label.setStyleSheet(_FLASH_STYLE)
        QTimer.singleShot(1500, lambda: label.setStyleSheet(_VALUE_STYLE))

    # -------------------------------------------------------------- ciclo de vida
    def closeEvent(self, event) -> None:  # noqa: N802 - método do Qt
        if self._running:
            self._stop()
        self._history.close()
        super().closeEvent(event)
