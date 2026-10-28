"""Arredondamento da leitura contínua para a tela principal.

O grosso da estabilidade vem do firmware (filtro interno do MPU6050 + filtro
adaptativo, ver firmware/src/AngleSensor.h); aqui é só a apresentação. Nada
disso afeta os dados guardados: histórico, mín/máx e relatório usam sempre o
ângulo bruto (0,01°).

Sem Qt de propósito: a mesma regra roda no app Android
(`android-app/.../ui/MainViewModel.kt`, `angleForDisplay`) e é testada sem
hardware em `sim/exibicao_test.py` (ver sim/run.sh).
"""

from __future__ import annotations

# Degraus de exibição disponíveis, por eixo. 0,25° é o padrão — o mesmo da
# versão 1, e as duas versões mostram o ângulo com a mesma granularidade.
# 0,1° é para bancada, comparando com uma referência de precisão (nível
# eletrônico, mesa giratória). Medindo a cadeia completa, o ruído do valor
# filtrado (~0,005°) é bem menor que meio degrau mesmo em 0,1°, então a
# leitura também não tremula nele.
DISPLAY_STEPS_DEG = (0.25, 0.1)
DEFAULT_DISPLAY_STEP_DEG = 0.25

# Histerese, como fração do degrau: sem ela, um valor parado bem na fronteira
# entre dois degraus (ex: 1,125° com degrau de 0,25°) alterna entre os dois a
# cada leitura. Só troca de degrau quando o valor passa da fronteira com uma
# margem extra. Proporcional ao degrau desde o app 2.2.0: com 0,05° fixos
# (era o valor para 0,25°), em 0,1° o valor tinha de andar um degrau inteiro
# para a tela mudar, e ela ficava até 0,1° atrás da leitura.
DISPLAY_HYSTERESIS_FRACTION = 0.2


def normalize_step(step_deg: float) -> float:
    """Degrau válido mais próximo (protege contra valor gravado inválido)."""
    return min(DISPLAY_STEPS_DEG, key=lambda s: abs(s - step_deg))


def display_decimals(step_deg: float) -> int:
    """Casas decimais coerentes com o degrau: 0,25° → 2; 0,1° → 1."""
    return 1 if abs(step_deg - 0.1) < 1e-9 else 2


def angle_for_display(angle_deg: float, current: float | None, step_deg: float) -> float:
    """Arredonda para o degrau, com histerese em relação ao valor exibido
    (`current`, None quando ainda não há). Devolve o novo valor exibido."""
    if current is None or abs(angle_deg - current) >= step_deg / 2 + DISPLAY_HYSTERESIS_FRACTION * step_deg:
        return round(round(angle_deg / step_deg) * step_deg, 6)
    return current


def format_angle(value_deg: float, step_deg: float) -> str:
    return f"{value_deg:.{display_decimals(step_deg)}f}°"


def short_step_label(step_deg: float) -> str:
    return "0,25°" if abs(step_deg - 0.25) < 1e-9 else "0,1°"


def step_label(step_deg: float) -> str:
    return "0,25° (padrão)" if abs(step_deg - 0.25) < 1e-9 else "0,1° (bancada, referência de precisão)"
