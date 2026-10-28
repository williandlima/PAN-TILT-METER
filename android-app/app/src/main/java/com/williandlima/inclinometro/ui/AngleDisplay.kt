package com.williandlima.inclinometro.ui

import kotlin.math.abs

/**
 * Arredondamento da leitura contínua para a tela principal — a mesma regra
 * do app desktop (`python-app/ui/angle_display.py`, testada em
 * `sim/exibicao_test.py`). O grosso da estabilidade vem do firmware (filtro
 * interno do MPU6050 + filtro adaptativo, ver firmware/src/AngleSensor.h);
 * aqui é só a apresentação. Nada disso afeta os dados guardados — histórico,
 * mín/máx e relatório usam sempre o ângulo bruto (0,01°).
 *
 * Sem dependência de Android de propósito: lógica pura.
 */
object AngleDisplay {
    /**
     * Degraus de exibição disponíveis, por eixo. 0,25° é o padrão — o mesmo da
     * versão 1. 0,1° é para bancada, comparando com uma referência de precisão
     * (nível eletrônico, mesa giratória). O ruído do valor filtrado (~0,005°)
     * é bem menor que meio degrau mesmo em 0,1°: a leitura não tremula.
     */
    val STEPS_DEG = listOf(0.25, 0.1)
    const val DEFAULT_STEP_DEG = 0.25

    /**
     * Histerese, como fração do degrau: sem ela, um valor parado na fronteira
     * entre dois degraus (ex: 1,125° com degrau de 0,25°) alterna entre os
     * dois a cada leitura. Proporcional ao degrau desde o app 2.2.0: com 0,05°
     * fixos, em 0,1° o valor tinha de andar um degrau inteiro para a tela
     * mudar, e ela ficava até 0,1° atrás da leitura.
     */
    const val HYSTERESIS_FRACTION = 0.2

    /** Degrau válido mais próximo (protege contra valor gravado inválido). */
    fun normalize(stepDeg: Double): Double = STEPS_DEG.minByOrNull { abs(it - stepDeg) } ?: DEFAULT_STEP_DEG

    /** Casas decimais coerentes com o degrau: 0,25° → 2; 0,1° → 1. */
    fun decimals(stepDeg: Double): Int = if (abs(stepDeg - 0.1) < 1e-9) 1 else 2

    /** Novo valor exibido, com histerese em relação ao atual (null = nenhum). */
    fun forDisplay(angleDeg: Double, current: Double?, stepDeg: Double): Double {
        if (current != null && abs(angleDeg - current) < stepDeg / 2 + HYSTERESIS_FRACTION * stepDeg) {
            return current
        }
        return Math.round(angleDeg / stepDeg) * stepDeg
    }

    fun format(valueDeg: Double, stepDeg: Double): String = "%.${decimals(stepDeg)}f°".format(valueDeg)

    fun shortLabel(stepDeg: Double): String = if (abs(stepDeg - 0.25) < 1e-9) "0,25°" else "0,1°"

    fun label(stepDeg: Double): String =
        if (abs(stepDeg - 0.25) < 1e-9) "0,25° (padrão)" else "0,1° (bancada, referência de precisão)"
}
