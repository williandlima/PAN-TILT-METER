"""Calibração da ESCALA do pan (giroscópio) a partir de giros de referência.

Por que existe (seção "Pan: reavaliação do método" de firmware/README.md):
depois que o ZUPT resolve o bias, o erro dominante do pan num giro é a
escala do giroscópio. O firmware usava UM número (PAN_SCALE_CORRECTION =
1,030), tirado de 2 pontos de 8° e 9°. Mas a escala efetiva ao longo do eixo
de pan é nᵀ·M·n — M é a matriz de sensibilidade do giro (±3% por eixo, ±2%
de sensibilidade cruzada no MPU6050) e n o eixo de pan nos eixos do chip,
que muda com o tilt. No plano do tilt isso vale EXATAMENTE

    k(t) = A + B·cos 2t + C·sin 2t        (t = tilt bruto)

e MEMS costumam ter ainda uma assimetria de escala entre os dois sentidos.
Esta ferramenta ajusta A, B, C e a assimetria por mínimos quadrados e
imprime PAN_SCALE_CORRECTION/_COS2/_SIN2/_ASYM para firmware/src/Config.h.

Entrada: CSV (";" ou ",") com uma linha por giro:
    tilt        — tilt BRUTO durante o giro (°): coluna "tilt" do
                  tools/diagnostico_pan.py
    referencia  — quanto o pan girou de verdade (°, com sinal; + = horário
                  visto de cima): PTU-D300E pelo encoder (comandos PP,
                  resolução de milésimos de grau) ou mesa giratória
    leitura     — quanto o PAN-TILT METER mostrou que girou (°, com sinal):
                  diferença do pan antes e depois do giro

Faça giros grandes (±45° a ±90°; até a 1.6.27 a leitura saturava em ±90°) nos DOIS sentidos e em vários tilts (ex.:
−30°, 0°, 30°, 60°): giro pequeno mede mal a escala — foi o problema dos 2
pontos de 8°/9°. Informe com --fw-* as constantes que o firmware tinha
durante as medidas (padrão: as da 1.6.28 a 1.6.32, 1,027/0/0/0). A versão da placa aparece no app
("Testar conexão"); com dados de outra versão, passe as constantes DELA: 1.6.25 = 1,0487;
1.6.26 = 1,0466 e assimetria −0,0059; 1.6.27 = 1,0425; até a 1.6.24 = 1,030.

Uso:
    python tools/calibra_pan.py giros.csv
    python tools/calibra_pan.py --autoteste
"""
from __future__ import annotations

import argparse
import csv
import math
import sys

import numpy as np


def k_modelo(t_deg, a, b, c, asym, sentido):
    t2 = np.radians(2 * np.asarray(t_deg, dtype=float))
    return (a + b * np.cos(t2) + c * np.sin(t2)) * (1 + asym * np.sign(sentido))


def ajusta(tilt, ref, leit, fw):
    """k que o firmware deveria ter em cada giro = k_fw · ref/leitura; ajusta
    A + B·cos2t + C·sin2t + D·sinal (D = A·assimetria, linearizado)."""
    k_fw = k_modelo(tilt, *fw, ref)
    k_need = k_fw * ref / leit
    t2 = np.radians(2 * tilt)
    X = np.column_stack([np.ones_like(tilt), np.cos(t2), np.sin(t2), np.sign(ref)])
    w = np.abs(ref)  # giro maior mede melhor a escala
    coef = np.linalg.lstsq(X * w[:, None], k_need * w, rcond=None)[0]
    a, b, c, d = coef
    # Só identifica B e C com tilts variados; sem eles, fica só A (e a assimetria).
    if np.ptp(tilt) < 20:
        Xa = np.column_stack([np.ones_like(tilt), np.sign(ref)])
        a, d = np.linalg.lstsq(Xa * w[:, None], k_need * w, rcond=None)[0]
        b = c = 0.0
    return float(a), float(b), float(c), float(d / a)


def resumo(nome, ref, erro):
    for lado, m in (("horário (+)", ref > 0), ("anti-horário (−)", ref < 0)):
        if m.any():
            e = erro[m]
            rel = 100 * np.abs(e / ref[m])
            print(f"  {nome:30s} {lado:17s} máx {np.abs(e).max():5.2f}°  rms {math.sqrt(np.mean(e**2)):5.2f}°  "
                  f"(até {rel.max():.2f}% do giro)")


def analisa(tilt, ref, leit, fw):
    a, b, c, asym = ajusta(tilt, ref, leit, fw)
    novo = leit * k_modelo(tilt, a, b, c, asym, ref) / k_modelo(tilt, *fw, ref)
    print(f"Giros: {ref.size} ({int((ref > 0).sum())} horários, {int((ref < 0).sum())} anti-horários), "
          f"tilt de {tilt.min():+.1f}° a {tilt.max():+.1f}°")
    print(f"Escala ajustada: k(t) = {a:.4f} {b:+.4f}·cos2t {c:+.4f}·sin2t, assimetria {asym * 100:+.2f}%")
    for t in (-30, 0, 30, 60):
        print(f"   tilt {t:+3d}°: k = {k_modelo(t, a, b, c, 0, 1):.4f}")
    print("\nErro do giro (leitura − referência):")
    resumo("firmware durante a medida", ref, leit - ref)
    resumo("com as constantes ajustadas", ref, novo - ref)
    print("\nGiro a giro:   tilt   referência   antes    depois")
    for t, r, l, n in zip(tilt, ref, leit, novo):
        print(f"            {t:+6.1f}°  {r:+8.2f}°  {l - r:+6.2f}°  {n - r:+6.3f}°")
    print("\nConstantes para firmware/src/Config.h:")
    print(f"constexpr float PAN_SCALE_CORRECTION = {a:.4f}f;")
    print(f"constexpr float PAN_SCALE_COS2 = {b:.4f}f;")
    print(f"constexpr float PAN_SCALE_SIN2 = {c:.4f}f;")
    print(f"constexpr float PAN_SCALE_ASYM = {asym:.4f}f;")
    return a, b, c, asym, leit - ref, novo - ref


def le_csv(caminho):
    with open(caminho, newline="", encoding="utf-8-sig") as f:
        amostra = f.readline()
        f.seek(0)
        linhas = list(csv.DictReader(f, delimiter=";" if ";" in amostra else ","))
    num = lambda s: float(str(s).replace(",", "."))  # noqa: E731
    return (np.array([num(l["tilt"]) for l in linhas]), np.array([num(l["referencia"]) for l in linhas]),
            np.array([num(l["leitura"]) for l in linhas]))


def autoteste() -> int:
    """Giroscópio com tolerâncias de fábrica sorteadas (±3% por eixo, ±2%
    cruzada, 0,6% de assimetria), placa montada torta (X a 44°), giros de ±45°
    e ±90° em 4 tilts, com 0,05° de ruído na leitura; o firmware mede com a
    escala única de hoje. O ajuste tem de derrubar o erro."""
    rng = np.random.default_rng(11)
    M = np.diag(1 + rng.uniform(-0.03, 0.03, 3)) @ (np.eye(3) + rng.uniform(-0.02, 0.02, (3, 3)) * (1 - np.eye(3)))
    asym_true = 0.006
    fw = (1.030, 0.0, 0.0, 0.0)
    tilt, ref, leit = [], [], []
    m = math.radians(44)
    for t in (-30, 0, 30, 60):
        n = np.array([math.sin(m), math.cos(m) * math.sin(math.radians(t)), math.cos(m) * math.cos(math.radians(t))])
        sens = (n @ M @ n) / 1.030  # o giro real é ~3% "curto", como o 1,030 supõe
        for r in (90, -90, 45, -45):
            s = sens * (1 + asym_true * np.sign(r))
            tilt.append(t)
            ref.append(r)
            leit.append(r * s * 1.030 + 0.05 * rng.standard_normal())
    tilt, ref, leit = map(np.array, (tilt, ref, leit))
    *_, antes, depois = analisa(tilt, ref, leit, fw)
    ok = np.abs(depois).max() < 0.2 and np.abs(antes).max() > 0.5
    print("\nAUTOTESTE:", "OK" if ok else "FALHOU")
    return 0 if ok else 1


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv", nargs="?")
    p.add_argument("--autoteste", action="store_true")
    p.add_argument("--fw-a", type=float, default=1.027,
                   help="PAN_SCALE_CORRECTION da versão gravada DURANTE a medida: 1,030 até a 1.6.24; "
                        "1,0487 na 1.6.25; 1,0466 na 1.6.26 (com --fw-asym -0.0059); 1,0425 na 1.6.27; "
                        "1,027 desde a 1.6.28")
    p.add_argument("--fw-cos2", type=float, default=0.0)
    p.add_argument("--fw-sin2", type=float, default=0.0)
    p.add_argument("--fw-asym", type=float, default=0.0, help="PAN_SCALE_ASYM durante a medida (-0,0059 só na 1.6.26)")
    a = p.parse_args()
    if a.autoteste:
        return autoteste()
    if not a.csv:
        p.print_help()
        return 2
    tilt, ref, leit = le_csv(a.csv)
    if ref.size < 4 or not (ref > 0).any() or not (ref < 0).any():
        print("Preciso de pelo menos 4 giros, nos DOIS sentidos.")
        return 2
    analisa(tilt, ref, leit, (a.fw_a, a.fw_cos2, a.fw_sin2, a.fw_asym))
    return 0


if __name__ == "__main__":
    sys.exit(main())
