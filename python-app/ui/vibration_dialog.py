"""Diálogos do Modo Vibração: configurar a captura (duração/taxa) e exibir
o resultado (estatísticas + opção de salvar relatório em PDF)."""
from __future__ import annotations

from typing import TYPE_CHECKING

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
)

if TYPE_CHECKING:
    from limits.vibration_stats import VibrationStats

# Espelham VIBRATION_MAX_RATE_HZ e VIBRATION_MAX_SAMPLES de
# firmware/src/Config.h — o firmware limita os dois de qualquer jeito, mas
# repetir aqui permite avisar o usuário ANTES da captura, em vez de deixá-lo
# descobrir a truncagem depois, no resultado.
MAX_RATE_HZ = 500
MAX_SAMPLES = 6000


class VibrationConfigDialog(QDialog):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Modo Vibração — Configurar captura")

        layout = QVBoxLayout(self)

        info = QLabel(
            "Posicione o pan-tilt na posição de referência (ex: 90°) e use "
            "\"Calibrar\" antes de iniciar, para que a variação registrada "
            "seja relativa a essa posição."
        )
        info.setWordWrap(True)
        layout.addWidget(info)

        form = QFormLayout()

        self.duration_spin = QSpinBox()
        self.duration_spin.setRange(5, 600)
        self.duration_spin.setValue(30)
        self.duration_spin.setSuffix(" s")
        form.addRow("Duração da captura:", self.duration_spin)

        self.rate_spin = QSpinBox()
        self.rate_spin.setRange(1, MAX_RATE_HZ)
        self.rate_spin.setValue(50)
        self.rate_spin.setSuffix(" Hz")
        self.rate_spin.valueChanged.connect(self._update_capacity_note)
        form.addRow("Taxa de amostragem:", self.rate_spin)
        self.duration_spin.valueChanged.connect(self._update_capacity_note)

        layout.addLayout(form)

        # O firmware trunca a captura em MAX_SAMPLES amostras por eixo (limite
        # de RAM do buffer). Antes isso acontecia em silêncio: pedindo 30s a
        # 500Hz, vinham 12s e nada avisava. Agora o limite é dito na hora de
        # configurar, não descoberto depois no relatório.
        self.capacity_note = QLabel()
        self.capacity_note.setWordWrap(True)
        layout.addWidget(self.capacity_note)
        self._update_capacity_note()

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _update_capacity_note(self) -> None:
        duration = self.duration_spin.value()
        rate = self.rate_spin.value()
        requested = duration * rate
        nyquist = rate / 2.0
        if requested > MAX_SAMPLES:
            max_duration = MAX_SAMPLES / rate
            self.capacity_note.setText(
                f"⚠ A {rate} Hz, o firmware captura no máximo {MAX_SAMPLES} amostras "
                f"(~{max_duration:.0f} s). A captura será truncada nesse limite, "
                f"e não nos {duration} s pedidos."
            )
            self.capacity_note.setStyleSheet("color: #c62828; font-weight: bold;")
        else:
            self.capacity_note.setText(
                f"{requested} amostras por eixo. Mede frequências até "
                f"{nyquist:.0f} Hz (Nyquist), com resolução de "
                f"{1.0 / duration:.2f} Hz."
            )
            self.capacity_note.setStyleSheet("color: #888;")

    def result_values(self) -> tuple[float, float]:
        return float(self.duration_spin.value()), float(self.rate_spin.value())


def _format_axis_summary(title: str, stats: "VibrationStats") -> str:
    if stats.dominant_freq_hz is not None:
        peak_line = (
            f"Frequência dominante: {stats.dominant_freq_hz:.2f} Hz "
            f"(amplitude {stats.dominant_amplitude_deg:.3f}°, "
            f"SNR {stats.dominant_snr_db:.1f} dB)"
        )
    else:
        peak_line = "Frequência dominante: nenhum pico confiável (sinal compatível com ruído)."

    return (
        f"<b>{title}</b><br>"
        f"Desvio padrão: {stats.std_dev_deg:.3f}° &nbsp;|&nbsp; "
        f"RMS: {stats.rms_deg:.3f}° &nbsp;|&nbsp; "
        f"Pico a pico: {stats.peak_to_peak_deg:.3f}°<br>"
        f"Mínimo / Máximo: {stats.min_deg:.3f}° / {stats.max_deg:.3f}°<br>"
        f"{peak_line}"
    )


class VibrationResultDialog(QDialog):
    def __init__(
        self,
        stats: "VibrationStats",
        pan_stats: "VibrationStats | None" = None,
        parent=None,
        on_save_csv=None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Modo Vibração — Resultado da captura")
        self._save_requested = False

        layout = QVBoxLayout(self)

        blocks = [
            f"Amostras: {stats.n_samples}  (duração efetiva: {stats.duration_s:.2f} s)",
            _format_axis_summary("Inclinação (tilt)", stats),
        ]
        if pan_stats is not None:
            blocks.append(_format_axis_summary("Azimute (pan)", pan_stats))
        else:
            blocks.append(
                "<b>Azimute (pan)</b><br>não capturado — firmware anterior à v1.3.0."
            )

        label = QLabel("<br><br>".join(blocks))
        label.setTextFormat(Qt.RichText)
        layout.addWidget(label)

        save_btn = QPushButton("Salvar relatório PDF...")
        save_btn.clicked.connect(self._on_save)
        layout.addWidget(save_btn)

        # CSV com as amostras cruas (diagnóstico de trancos e vibração). Não
        # fecha a janela: dá para salvar o CSV e depois o PDF.
        if on_save_csv is not None:
            csv_btn = QPushButton("Salvar CSV...")
            csv_btn.setToolTip("Amostras da captura (tilt, taxa e ângulo do pan) em CSV para Excel.")
            csv_btn.clicked.connect(on_save_csv)
            layout.addWidget(csv_btn)

        close_btn = QPushButton("Fechar")
        close_btn.clicked.connect(self.accept)
        layout.addWidget(close_btn)

    def _on_save(self) -> None:
        self._save_requested = True
        self.accept()

    @property
    def save_requested(self) -> bool:
        return self._save_requested
