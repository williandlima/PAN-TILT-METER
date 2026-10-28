"""Teste da exibição da leitura contínua (resolução de 0,25° ou 0,1° por eixo).

Duas partes, sem hardware:
  1. a regra pura de python-app/ui/angle_display.py — degrau, histerese e
     casas decimais (a mesma regra do app Android, MainViewModel.kt);
  2. a janela principal REAL do app desktop, em Qt sem tela (offscreen):
     seletor de resolução no cartão de cada eixo, redesenho na hora, os dois
     eixos independentes e a escolha lembrada na sessão seguinte.

Ver run.sh. Requer PyQt5 (python-app/requirements.txt).
"""

import os
import random
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "python-app"))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from ui.angle_display import (  # noqa: E402
    DISPLAY_HYSTERESIS_FRACTION,
    DISPLAY_STEPS_DEG,
    angle_for_display,
    format_angle,
)

failures = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global failures
    print(f"  {'OK    ' if ok else 'FALHOU'} {name}{': ' + detail if detail else ''}")
    if not ok:
        failures += 1


print("=== regra de exibição (ui/angle_display.py) ===")
for step in DISPLAY_STEPS_DEG:
    # Rampa lenta para cima e para baixo: a tela nunca se afasta da leitura
    # mais que meio degrau + histerese, e só mostra múltiplos do degrau.
    limit = step / 2 + DISPLAY_HYSTERESIS_FRACTION * step + 1e-9
    shown, worst, off_grid = None, 0.0, 0
    values = [i * 0.001 for i in range(-2000, 2001)] + [i * 0.001 for i in range(2000, -2001, -1)]
    for v in values:
        shown = angle_for_display(v, shown, step)
        worst = max(worst, abs(shown - v))
        if abs(shown / step - round(shown / step)) > 1e-6:
            off_grid += 1
    check(f"degrau {step}: atraso máximo da tela", worst <= limit, f"{worst:.3f}° (limite {limit:.3f}°)")
    check(f"degrau {step}: só múltiplos do degrau", off_grid == 0, f"{off_grid} fora")

    # Parado na fronteira entre dois degraus, com o ruído do valor filtrado
    # (~0,005°, 10x de folga aqui): a tela não pode alternar.
    rng = random.Random(1)
    boundary = 3 * step + step / 2
    shown, changes, prev = None, 0, None
    for _ in range(2000):
        shown = angle_for_display(boundary + rng.gauss(0, 0.05 * step), shown, step)
        if prev is not None and shown != prev:
            changes += 1
        prev = shown
    check(f"degrau {step}: parado na fronteira não tremula", changes == 0, f"{changes} trocas em 2000 leituras")

# Varredura de referência, repetida em sim/AngleDisplayTest.kt (app Android):
# os dois apps têm de mostrar exatamente os mesmos valores.
import math  # noqa: E402

for step, (e_acc, e_changes) in ((0.25, (2523.5, 107)), (0.1, (2493.6, 275))):
    shown, acc, changes = None, 0.0, 0
    for i in range(6000):
        new = angle_for_display(2.5 * math.sin(i * 0.003) + 0.37, shown, step)
        if shown is not None and new != shown:
            changes += 1
        shown = new
        acc += new
    check(f"degrau {step}: varredura de referência (mesma do Android)",
          abs(acc - e_acc) < 1e-3 and changes == e_changes, f"soma {acc:.4f} / {changes} trocas")

check("formato 0,25°", format_angle(12.25, 0.25) == "12.25°", format_angle(12.25, 0.25))
check("formato 0,1°", format_angle(12.3, 0.1) == "12.3°", format_angle(12.3, 0.1))
check("formato 0,1° negativo", format_angle(-0.1, 0.1) == "-0.1°", format_angle(-0.1, 0.1))

print("=== janela principal (app desktop, Qt offscreen) ===")
from PyQt5.QtCore import QSettings  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

# Configurações gravadas num diretório temporário, sem tocar nas do usuário.
settings_dir = tempfile.mkdtemp(prefix="inclinometro_exib_")
QSettings.setPath(QSettings.IniFormat, QSettings.UserScope, settings_dir)

app = QApplication.instance() or QApplication([])
from data_source.base import AngleReading  # noqa: E402
from limits.limit_tracker import PAN_AXIS, TILT_AXIS  # noqa: E402
from ui.main_window import MainWindow  # noqa: E402

w = MainWindow()
tilt, pan = w._axis_widgets[TILT_AXIS], w._axis_widgets[PAN_AXIS]
check("padrão 0,25° nos dois eixos", tilt["resolution"].currentData() == 0.25 and pan["resolution"].currentData() == 0.25)

w._on_reading(AngleReading(angle_deg=12.34, timestamp=time.time(), pan_deg=40.07))
check("tilt 12,34 em 0,25°", tilt["value"].text() == "12.25°", tilt["value"].text())
check("pan 40,07 em 0,25°", pan["value"].text() == "40.00°", pan["value"].text())

tilt["resolution"].setCurrentIndex(tilt["resolution"].findData(0.1))
check("tilt troca para 0,1° na hora", tilt["value"].text() == "12.3°", tilt["value"].text())
check("pan não muda com o tilt", pan["value"].text() == "40.00°", pan["value"].text())

w._on_reading(AngleReading(angle_deg=12.41, timestamp=time.time(), pan_deg=40.07))
check("tilt 12,41 em 0,1°", tilt["value"].text() == "12.4°", tilt["value"].text())

pan["resolution"].setCurrentIndex(pan["resolution"].findData(0.1))
check("pan troca para 0,1°", pan["value"].text() == "40.1°", pan["value"].text())
pan["resolution"].setCurrentIndex(pan["resolution"].findData(0.25))
check("pan volta para 0,25°", pan["value"].text() == "40.00°", pan["value"].text())

# Configurações > Resolução: o mesmo caminho de _open_settings() ao aceitar.
from ui.settings_dialog import SettingsDialog, save_display_steps  # noqa: E402

dialog = SettingsDialog(w._settings, w)
check("Configurações abre com a escolha do cartão", dialog.tilt_resolution_combo.currentData() == 0.1
      and dialog.pan_resolution_combo.currentData() == 0.25)
dialog.pan_resolution_combo.setCurrentIndex(dialog.pan_resolution_combo.findData(0.1))
w._settings = dialog.result_settings()
save_display_steps(w._settings)
for axis in (TILT_AXIS, PAN_AXIS):
    w._sync_display_step(axis)
check("Configurações atualiza o cartão do pan", pan["resolution"].currentData() == 0.1 and pan["value"].text() == "40.1°",
      pan["value"].text())
pan["resolution"].setCurrentIndex(pan["resolution"].findData(0.25))
w.close()

w2 = MainWindow()
check(
    "escolha lembrada na sessão seguinte",
    w2._axis_widgets[TILT_AXIS]["resolution"].currentData() == 0.1
    and w2._axis_widgets[PAN_AXIS]["resolution"].currentData() == 0.25
    and w2._settings.tilt_display_step_deg == 0.1,
)
w2.close()

print("EXIBIÇÃO: TUDO OK" if failures == 0 else "EXIBIÇÃO: HOUVE FALHA")
sys.exit(1 if failures else 0)
