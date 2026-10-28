// Teste da regra de exibição do app Android (android-app/.../ui/AngleDisplay.kt),
// compilada com kotlinc puro, sem o SDK do Android. Mesmos critérios de
// sim/exibicao_test.py e, no fim, a mesma varredura que o lado Python: os dois
// apps têm de mostrar exatamente os mesmos valores. Ver run.sh.
import com.williandlima.inclinometro.ui.AngleDisplay
import kotlin.math.abs
import kotlin.math.round
import kotlin.math.sin
import kotlin.system.exitProcess

var failures = 0

fun check(name: String, ok: Boolean, detail: String = "") {
    println("  ${if (ok) "OK    " else "FALHOU"} $name${if (detail.isNotEmpty()) ": $detail" else ""}")
    if (!ok) failures++
}

fun main() {
    println("=== regra de exibição do Android (AngleDisplay.kt) ===")
    for (step in AngleDisplay.STEPS_DEG) {
        val limit = step / 2 + AngleDisplay.HYSTERESIS_FRACTION * step + 1e-9
        var shown: Double? = null
        var worst = 0.0
        var offGrid = 0
        val values = (-2000..2000).map { it * 0.001 } + (2000 downTo -2000).map { it * 0.001 }
        for (v in values) {
            val s = AngleDisplay.forDisplay(v, shown, step)
            shown = s
            worst = maxOf(worst, abs(s - v))
            if (abs(s / step - round(s / step)) > 1e-6) offGrid++
        }
        check("degrau $step: atraso máximo da tela", worst <= limit, "%.3f° (limite %.3f°)".format(worst, limit))
        check("degrau $step: só múltiplos do degrau", offGrid == 0, "$offGrid fora")
    }
    check("casas decimais", AngleDisplay.decimals(0.25) == 2 && AngleDisplay.decimals(0.1) == 1)
    check("degrau gravado inválido vira o mais próximo", AngleDisplay.normalize(0.12) == 0.1 && AngleDisplay.normalize(3.0) == 0.25)

    // Mesma varredura de sim/exibicao_test.py (sinal lento + deslocamento):
    // soma dos valores exibidos e número de trocas de degrau, por degrau.
    val expected = mapOf(0.25 to Pair(2523.5, 107), 0.1 to Pair(2493.6, 275))
    for (step in AngleDisplay.STEPS_DEG) {
        var shown: Double? = null
        var acc = 0.0
        var changes = 0
        for (i in 0 until 6000) {
            val v = 2.5 * sin(i * 0.003) + 0.37
            val s = AngleDisplay.forDisplay(v, shown, step)
            if (shown != null && s != shown) changes++
            shown = s
            acc += s
        }
        val (eAcc, eChanges) = expected.getValue(step)
        check(
            "degrau $step: igual ao app desktop",
            abs(acc - eAcc) < 1e-3 && changes == eChanges,
            "soma %.4f / %d trocas (desktop %.4f / %d)".format(acc, changes, eAcc, eChanges),
        )
    }
    println(if (failures == 0) "EXIBIÇÃO ANDROID: TUDO OK" else "EXIBIÇÃO ANDROID: HOUVE FALHA")
    if (failures > 0) exitProcess(1)
}
