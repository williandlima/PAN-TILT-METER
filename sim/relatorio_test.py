"""Testes do relatório PDF e da aquisição que o alimenta — sem hardware.

Cobre só as classes envolvidas no relatório:
  1. LimitTracker (limits/limit_tracker.py): mín/máx por eixo e a origem de
     cada extremo (ESP32 a 100 Hz ou calculado pelo app);
  2. HistoryStore (limits/history_store.py): aquisições, leituras, eventos,
     tempo programado e migração de um banco criado por versão anterior;
  3. resumo da aquisição (report/acquisition.py): duração, taxa,
     interrupções, cobertura, aquisição em andamento;
  4. geração do PDF (report/report_generator.py): conteúdo conferido com
     pdftotext — sessão normal, sem azimute, vazia, com interrupção, longa,
     em andamento, e o relatório de vibração;
  5. a janela principal REAL em Qt sem tela: aquisição com tempo programado
     para sozinha no fim, "Resetar limites" abre aquisição nova e o
     relatório sai da aquisição certa.

Ver run.sh. Requer PyQt5, reportlab, matplotlib e numpy
(python-app/requirements.txt); a conferência do texto do PDF usa o
pdftotext (poppler-utils) se houver.
"""

import os
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "python-app"))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from data_source.base import AngleReading  # noqa: E402
from limits.history_store import HistoryStore, SessionInfo  # noqa: E402
from limits.limit_tracker import (  # noqa: E402
    PAN_AXIS,
    SOURCE_APP,
    SOURCE_FIRMWARE,
    TILT_AXIS,
    LimitEvent,
    LimitTracker,
)
from report.acquisition import (  # noqa: E402
    format_clock,
    format_duration,
    format_planned,
    summarize_acquisition,
)
from report.report_generator import (  # noqa: E402
    _axis_extremes,
    _time_scale,
    generate_report,
    generate_vibration_report,
)

failures = 0
TMP = tempfile.mkdtemp(prefix="relatorio_test_")
PDFTOTEXT = shutil.which("pdftotext")


def check(name: str, ok: bool, detail: str = "") -> None:
    global failures
    print(f"  {'OK    ' if ok else 'FALHOU'} {name}{': ' + detail if detail else ''}")
    if not ok:
        failures += 1


def pdf_text(path: str) -> str | None:
    if not PDFTOTEXT:
        return None
    return subprocess.run([PDFTOTEXT, "-layout", path, "-"], capture_output=True, text=True).stdout


def series(t0: float, seconds: float, rate_hz: float = 4.0, gap: tuple[float, float] | None = None,
           pan: bool = True, amp: float = 2.0) -> list[AngleReading]:
    """Leituras sintéticas: senoide lenta nos dois eixos, opcionalmente com
    um buraco (gap) de [início, fim) segundos sem leitura."""
    import math

    out = []
    n = int(seconds * rate_hz) + 1
    for i in range(n):
        t = i / rate_hz
        if gap and gap[0] <= t < gap[1]:
            continue
        out.append(
            AngleReading(
                angle_deg=amp * math.sin(2 * math.pi * t / 60),
                timestamp=t0 + t,
                pan_deg=(10 + amp * math.cos(2 * math.pi * t / 60)) if pan else None,
            )
        )
    return out


# ---------------------------------------------------------------------------
print("=== LimitTracker: mín/máx e origem do extremo ===")
tr = LimitTracker(TILT_AXIS)
ev = tr.process(AngleReading(1.0, 100.0))
check("primeira leitura gera mín e máx", sorted(e.kind for e in ev) == ["max", "min"])
check("sem extremos do firmware: origem = app", all(e.source == SOURCE_APP for e in ev))
check("leitura dentro da faixa não gera evento", tr.process(AngleReading(1.0, 100.2)) == [])
ev = tr.process(AngleReading(0.5, 100.4, angle_min_deg=-3.0, angle_max_deg=1.2))
kinds = {e.kind: e for e in ev}
check("extremo do firmware vira o valor do evento",
      kinds.get("min") is not None and kinds["min"].value_deg == -3.0 and kinds["max"].value_deg == 1.2)
check("extremo do firmware: origem = firmware", all(e.source == SOURCE_FIRMWARE for e in ev))
check("evento do firmware ancorado na leitura que o trouxe", kinds["min"].reading.timestamp == 100.4)
tr.reset()
check("reset zera os extremos", tr.min_reading is None and tr.max_reading is None)
pan_tr = LimitTracker(PAN_AXIS)
check("eixo de pan ignora leitura sem pan", pan_tr.process(AngleReading(1.0, 1.0, pan_deg=None)) == [])
ev = pan_tr.process(AngleReading(1.0, 1.0, pan_deg=-5.0))
check("eixo de pan usa pan_deg", all(e.value_deg == -5.0 and e.axis == PAN_AXIS for e in ev) and len(ev) == 2)

# ---------------------------------------------------------------------------
print("=== HistoryStore: aquisições, eventos e migração ===")
db = os.path.join(TMP, "hist.db")
store = HistoryStore(db)
sid = store.start_session("ble", planned_duration_s=600)
for r in series(time.time(), 5):
    store.add_reading(r)
store.add_limit_event(LimitEvent("min", AngleReading(-2.5, time.time(), pan_deg=3.0), TILT_AXIS, SOURCE_FIRMWARE))
store.add_limit_event(LimitEvent("max", AngleReading(1.0, time.time(), pan_deg=12.0), PAN_AXIS, SOURCE_APP))
store.end_session()
info = store.get_session(sid)
check("aquisição guarda o tempo programado", info is not None and info.planned_duration_s == 600)
check("aquisição encerrada tem fim", info.ended_at is not None and info.ended_at >= info.started_at)
check("leituras gravadas", store.count_readings(sid) == len(store.get_readings(sid)) == 21)
evs = store.get_limit_events(sid)
check("origem do evento volta do banco", [e.source for e in evs] == [SOURCE_FIRMWARE, SOURCE_APP])
check("eixo e valor do evento voltam do banco", evs[1].axis == PAN_AXIS and evs[1].value_deg == 12.0)
sid_empty = store.start_session("ble")  # "Resetar limites" e logo "Parar": aquisição vazia por último
store.end_session()
check("aquisição livre: tempo programado nulo", store.get_session(sid_empty).planned_duration_s is None)
latest = store.latest_session_with_readings()
check("relatório pula a aquisição vazia mais recente", latest is not None and latest.id == sid,
      f"pegou {latest.id if latest else None}, esperado {sid}")
check("list_sessions: mais recente primeiro", store.list_sessions()[0].id == sid_empty)
store.close()

old_db = os.path.join(TMP, "antigo.db")
con = sqlite3.connect(old_db)
con.executescript(
    """
    CREATE TABLE sessions (id INTEGER PRIMARY KEY AUTOINCREMENT, started_at REAL NOT NULL,
                           ended_at REAL, mode TEXT NOT NULL);
    CREATE TABLE readings (session_id INTEGER NOT NULL, timestamp REAL NOT NULL, angle_deg REAL NOT NULL);
    CREATE TABLE limit_events (session_id INTEGER NOT NULL, kind TEXT NOT NULL,
                               angle_deg REAL NOT NULL, timestamp REAL NOT NULL);
    INSERT INTO sessions VALUES (1, 1000.0, 1060.0, 'real');
    INSERT INTO readings VALUES (1, 1000.5, 1.5);
    INSERT INTO limit_events VALUES (1, 'max', 1.5, 1000.5);
    """
)
con.commit()
con.close()
try:
    old = HistoryStore(old_db)
    s_old = old.get_session(1)
    e_old = old.get_limit_events(1)
    check("banco antigo abre e migra", s_old is not None and s_old.planned_duration_s is None)
    check("evento antigo: eixo tilt e origem app",
          len(e_old) == 1 and e_old[0].axis == TILT_AXIS and e_old[0].source == SOURCE_APP)
    check("leitura antiga: pan nulo", old.get_readings(1)[0].pan_deg is None)
    old.close()
except Exception as exc:  # noqa: BLE001
    check("banco antigo abre e migra", False, repr(exc))

# ---------------------------------------------------------------------------
print("=== HistoryStore: limpeza do histórico (retenção) e versões ===")
DAY = 86400.0
pdb = os.path.join(TMP, "prune.db")
ps = HistoryStore(pdb)
agora = time.time()


def aquisicao(dias_atras: float, n: int, com_evento: bool = True) -> int:
    sid_ = ps.start_session("ble", app_version="2.3.0")
    t0_ = agora - dias_atras * DAY
    ps._conn.execute("UPDATE sessions SET started_at = ? WHERE id = ?", (t0_, sid_))
    for i_ in range(n):
        ps.add_reading(AngleReading(1.0, t0_ + i_ * 0.25, pan_deg=2.0))
    if com_evento and n:
        ps.add_limit_event(LimitEvent("max", AngleReading(1.0, t0_, pan_deg=2.0), TILT_AXIS, SOURCE_APP))
    ps.end_session()
    ps._conn.execute("UPDATE sessions SET ended_at = ? WHERE id = ?", (t0_ + n * 0.25, sid_))
    return sid_


velha = aquisicao(200, 4000)
media = aquisicao(100, 4000)
nova = aquisicao(5, 400)
vazia = aquisicao(1, 0)
cap_velha = ps.save_vibration_capture("ble", 10, 100, series(agora - 200 * DAY, 10, rate_hz=100))
ps._conn.execute("UPDATE vibration_captures SET started_at = ? WHERE id = ?", (agora - 200 * DAY, cap_velha))
cap_nova = ps.save_vibration_capture("ble", 2, 100, series(agora, 2, rate_hz=100))
atual = ps.start_session("ble", app_version="2.3.0")  # em andamento, sem leituras ainda
ps._conn.execute("UPDATE sessions SET started_at = ? WHERE id = ?", (agora - 400 * DAY, atual))
ps.flush()
tam_antes = ps.size_bytes()

r_none = ps.prune(None)
check("'manter sempre' só remove aquisições vazias encerradas",
      r_none["sessions"] == 0 and r_none["empty_sessions"] == 1 and ps.get_session(vazia) is None
      and ps.count_readings(velha) == 4000, f"{r_none}")
r90 = ps.prune(90)
check("90 dias: remove as de 200 e 100 dias", r90["sessions"] == 2 and r90["readings"] == 8000, f"{r90}")
check("90 dias: aquisição recente intacta", ps.count_readings(nova) == 400 and len(ps.get_limit_events(nova)) == 1)
check("90 dias: leituras e eventos das antigas apagados",
      ps.count_readings(velha) == 0 and ps.get_limit_events(media) == [] and ps.get_session(velha) is None)
check("90 dias: captura de vibração antiga apagada, recente mantida",
      r90["vibration_captures"] == 1 and ps.get_vibration_samples(cap_velha) == []
      and len(ps.get_vibration_samples(cap_nova)) > 0)
check("aquisição em andamento nunca é apagada (mesmo 'antiga' e vazia)",
      ps.get_session(atual) is not None and ps.current_session_id == atual)
check("arquivo diminui depois da limpeza (VACUUM)", ps.size_bytes() < tam_antes and r90["freed_bytes"] > 0,
      f"{tam_antes} -> {ps.size_bytes()} bytes")
check("limpar de novo não remove nada", ps.prune(90)["sessions"] == 0)
ps.add_reading(AngleReading(1.0, time.time()))
check("gravação continua funcionando depois da limpeza", ps.count_readings(atual) == 1)
ps.set_firmware_version("1.6.21")
ps.end_session()
info_v = ps.get_session(atual)
check("aquisição guarda as versões do app e do firmware",
      info_v.app_version == "2.3.0" and info_v.firmware_version == "1.6.21", f"{info_v}")
st = ps.stats()
check("stats() resume o histórico", st["sessions"] == 2 and st["readings"] == 401 and st["size_bytes"] > 0, f"{st}")

# Gravação em lote: leituras ficam visíveis na hora e chegam ao disco no flush/close.
ps.start_session("ble")
for i in range(10):
    ps.add_reading(AngleReading(float(i), time.time()))
check("leituras em lote visíveis antes do commit", ps.count_readings(ps.current_session_id) == 10)
ps.close()
con2 = sqlite3.connect(pdb)
check("close() grava o lote pendente em disco",
      con2.execute("SELECT COUNT(*) FROM readings").fetchone()[0] == 411)
con2.close()

# ---------------------------------------------------------------------------
print("=== resumo da aquisição (report/acquisition.py) ===")
T0 = 1_700_000_000.0
readings = series(T0 + 2.0, 598.0, rate_hz=4.0, gap=(200.0, 230.0))  # 1ª leitura 2 s após o início
session = SessionInfo(1, T0, T0 + 600.0, "ble", 600.0, "1.6.21", "2.3.0")
events = [LimitEvent("min", readings[10], TILT_AXIS, SOURCE_APP),
          LimitEvent("max", readings[20], TILT_AXIS, SOURCE_FIRMWARE)]
acq = summarize_acquisition(session, readings, events)
check("duração = início ao fim", abs(acq.duration_s - 600.0) < 1e-6, format_duration(acq.duration_s))
check("taxa média ~4 leituras/s (descontado o buraco)", 3.7 < acq.mean_rate_hz < 4.0, f"{acq.mean_rate_hz:.2f}")
check("intervalo típico 250 ms", abs(acq.median_interval_s - 0.25) < 1e-6)
check("uma interrupção de ~30 s", len(acq.gaps) == 1 and 29.9 < acq.gaps[0].duration_s < 30.5,
      f"{[round(g.duration_s, 2) for g in acq.gaps]}")
check("cobertura ~94,7 % (2 s até conectar + 30 s de buraco)", 94.0 < acq.coverage_pct < 95.5,
      f"{acq.coverage_pct:.1f} %")
check("aquisição programada concluída", acq.completed_planned is True)
check("origem dos extremos: firmware no tilt, nada no pan",
      acq.extremes_source[TILT_AXIS] == SOURCE_FIRMWARE and acq.extremes_source[PAN_AXIS] is None)
running = summarize_acquisition(SessionInfo(2, T0, None, "ble", 600.0), readings[:100])
check("em andamento: fim = última leitura", running.in_progress and running.end_reference == readings[99].timestamp)
check("em andamento: programada ainda não concluída", running.completed_planned is False)
empty = summarize_acquisition(SessionInfo(3, T0, None, "ble"), [])
check("aquisição vazia não quebra", empty.n_readings == 0 and empty.mean_rate_hz is None and empty.duration_s == 0)
check("format_duration", (format_duration(45), format_duration(303), format_duration(3725))
      == ("45 s", "5 min 03 s", "1 h 02 min 05 s"))
check("format_planned", (format_planned(None).startswith("Livre"), format_planned(600), format_planned(3600))
      == (True, "10 min", "1 h"))
check("format_clock", (format_clock(192), format_clock(3725)) == ("03:12", "1:02:05"))

# ---------------------------------------------------------------------------
print("=== extremos do relatório (_axis_extremes) ===")
evs = [LimitEvent("min", AngleReading(-4.0, 1.0), TILT_AXIS, SOURCE_FIRMWARE),
       LimitEvent("max", AngleReading(3.0, 2.0), TILT_AXIS, SOURCE_FIRMWARE)]
check("prefere os eventos (medidos a 100 Hz) às leituras",
      _axis_extremes([AngleReading(-1.0, 1.0), AngleReading(1.0, 2.0)], evs, TILT_AXIS) == (-4.0, 3.0))
check("sem eventos: cai para as leituras",
      _axis_extremes([AngleReading(-1.0, 1.0), AngleReading(1.0, 2.0)], [], TILT_AXIS) == (-1.0, 1.0))
check("eixo sem dado: (None, None)", _axis_extremes([AngleReading(1.0, 1.0)], [], PAN_AXIS) == (None, None))

# ---------------------------------------------------------------------------
print("=== geração do PDF (report/report_generator.py) ===")
if not PDFTOTEXT:
    print("  (pdftotext não encontrado: confere só que os PDFs são gerados)")


def make_pdf(name, session_, readings_, events_, must_have=()):
    path = os.path.join(TMP, name)
    try:
        generate_report(path, session_, readings_, events_)
    except Exception as exc:  # noqa: BLE001
        check(f"{name}: gera", False, repr(exc))
        return
    ok = os.path.getsize(path) > 1000
    text = pdf_text(path)
    missing = [m for m in must_have if text is not None and m not in text]
    check(f"{name}: gera" + (" e contém o esperado" if text is not None else ""), ok and not missing,
          f"faltando: {missing}" if missing else "")
    return text


text = make_pdf(
    "normal.pdf", session, readings, events,
    must_have=["Aquisição dos dados", "Como os dados foram adquiridos", "Duração da aquisição",
               "Versões (app / firmware)", "app 2.3.0 / firmware 1.6.21",
               "10 min 00 s", "10 min", "concluída", "Taxa média de leitura", "250 ms",
               "Interrupções", "1 — 30 s", "Cobertura da aquisição", "medidos no ESP32 a 100 Hz",
               "ESP32 (100 Hz)", "App (leituras)"],
)
make_pdf("sem_azimute.pdf", SessionInfo(4, T0, T0 + 60, "real"), series(T0, 60, pan=False), [],
         must_have=["não registrado (firmware sem eixo de azimute)", "Livre", "Interrupções", "nenhuma",
                      "calculados a partir das leituras"])
make_pdf("vazia.pdf", SessionInfo(5, T0, None, "ble"), [], [],
         must_have=["nenhuma leitura ainda", "Nenhum evento de limite registrado nesta aquisição"])
make_pdf("em_andamento.pdf", SessionInfo(6, T0, None, "ble", 1800.0), readings[:400], [],
         must_have=["em andamento — dados até", "30 min — em andamento"])
make_pdf("interrompida.pdf", SessionInfo(7, T0, T0 + 120, "ble", 600.0), series(T0, 120), [],
         must_have=["interrompida antes do fim", "2 min 00 s"])
make_pdf("longa.pdf", SessionInfo(8, T0, T0 + 4 * 3600, "ble"), series(T0, 4 * 3600, rate_hz=0.2), [],
         must_have=["4 h 00 min 00 s"])
check("eixo de tempo do gráfico: s / min / h conforme a duração",
      [_time_scale(d)[0] for d in (120, 600, 4 * 3600)] == [1.0, 60.0, 3600.0])
make_pdf("sem_sessao.pdf", None, series(T0, 30), [], must_have=["Aquisição dos dados"])

try:
    import numpy as np  # noqa: F401
    from limits.vibration_stats import analyze_axis

    vib = series(T0, 10, rate_hz=100, amp=0.5)
    vpath = os.path.join(TMP, "vibracao.pdf")
    generate_vibration_report(vpath, None, vib, analyze_axis(vib, 100.0, TILT_AXIS), None)
    vt = pdf_text(vpath)
    check("relatório de vibração continua gerando", os.path.getsize(vpath) > 1000 and
          (vt is None or "Relatório de Captura de Vibração" in vt))
except Exception as exc:  # noqa: BLE001
    check("relatório de vibração continua gerando", False, repr(exc))

# CSV da captura de vibração (botão "Salvar CSV..."): amostras cruas, com a
# TAXA do pan (°/s) — é ela que mostra se um degrau é oscilação ou giro real.
try:
    from data_source.base import build_vibration_readings
    from limits.history_export import export_vibration_csv

    rates = [0.0] * 50 + [100.0] * 10 + [0.0] * 40  # 10 amostras a 100°/s em 100 Hz = 10°
    caps = build_vibration_readings([0.5] * 100, rates, 100.0, T0)
    cpath = os.path.join(TMP, "captura.csv")
    n = export_vibration_csv(cpath, caps, 100.0, "firmware: 1.6.32")
    linhas = open(cpath, encoding="utf-8-sig").read().splitlines()
    dados = [l for l in linhas if not l.startswith("#")]
    check("CSV de vibração: uma linha por amostra + cabeçalho", n == 100 and len(dados) == 101, f"{len(dados)} linhas")
    check("CSV de vibração: colunas e vírgula decimal",
          dados[0] == "amostra;t_s;tilt_graus;pan_taxa_graus_s;pan_integrado_graus" and dados[56].split(";")[3] == "100,00",
          dados[56])
    check("CSV de vibração: integral pura da taxa (sem tirar tendência)", abs(float(dados[-1].split(";")[4].replace(",", ".")) - 10.0) < 0.01,
          dados[-1])
    check("CSV de vibração: cabeçalho com taxa e firmware",
          linhas[1].startswith("# taxa: 100,0 Hz; amostras: 100") and "firmware: 1.6.32" in linhas[2], linhas[1])
except Exception as exc:  # noqa: BLE001
    check("CSV de vibração", False, repr(exc))

# ---------------------------------------------------------------------------
print("=== janela principal: aquisição com tempo programado (Qt offscreen) ===")
try:
    from PyQt5.QtWidgets import QApplication, QMessageBox

    app = QApplication.instance() or QApplication(sys.argv)
    import ui.main_window as mw

    ui_db = os.path.join(TMP, "ui.db")
    mw.HistoryStore = lambda: HistoryStore(ui_db)
    asked: list[str] = []

    def fake_question(_parent, title, text, *args, **kwargs):
        asked.append(title)
        return QMessageBox.No

    mw.QMessageBox.question = staticmethod(fake_question)
    w = mw.MainWindow()
    w._settings.mode = "simulado"

    def pump(seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            app.processEvents()
            time.sleep(0.02)

    check("padrão: aquisição livre", w._planned_duration_s() is None)
    w.acq_duration_combo.addItem("3 s (teste)", 3.0)
    w.acq_duration_combo.setCurrentIndex(w.acq_duration_combo.count() - 1)
    w._start()
    check("seletor travado durante a aquisição", not w.acq_duration_combo.isEnabled())
    pump(1.2)
    check("cronômetro na tela", "gravando" in w.acq_status_label.text() and "/ 00:03" in w.acq_status_label.text(),
          w.acq_status_label.text())
    first_sid = w._history.current_session_id
    w._reset_limits()
    second_sid = w._history.current_session_id
    check("Resetar limites abre aquisição nova", second_sid != first_sid and second_sid is not None)
    pump(4.5)
    check("aquisição programada para sozinha no fim", not w._running)
    check("oferece gerar o relatório", "Aquisição concluída" in asked, f"{asked}")
    s2 = w._history.get_session(second_sid)
    check("aquisição gravada com o tempo programado e o fim",
          s2.planned_duration_s == 3.0 and s2.ended_at is not None and 2.5 <= s2.ended_at - s2.started_at <= 5.0,
          f"{s2.ended_at - s2.started_at:.1f} s" if s2.ended_at else "sem fim")
    check("leituras contadas na tela", w._history.count_readings(second_sid) > 0)
    check("rótulo mostra a última aquisição", w.acq_status_label.text().startswith("última:"),
          w.acq_status_label.text())

    out_pdf = os.path.join(TMP, "ui.pdf")
    saved: list[tuple[str, str]] = []
    w._ask_report_path = lambda *a, **k: out_pdf
    w._report_saved = lambda path, detail="": saved.append((path, detail))
    w._generate_report()
    check("relatório sai da última aquisição", bool(saved) and os.path.exists(out_pdf), f"{saved}")
    t = pdf_text(out_pdf)
    check("PDF da janela traz o tempo programado", t is None or "3 s" in t)

    # Tela de Histórico: lista, PDF de uma aquisição antiga, CSV e apagar.
    from ui.history_dialog import HistoryDialog
    resumos = w._history.list_session_summaries()
    check("histórico lista só aquisições com leituras, mais recente primeiro",
          len(resumos) == 2 and resumos[0]["session"].id == second_sid and resumos[0]["n_readings"] > 0,
          f"{[(r['session'].id, r['n_readings']) for r in resumos]}")
    hd = HistoryDialog(w._history, on_report=lambda sid: w._generate_report(w._history.get_session(sid)),
                       on_export=w._export_session_csv, on_delete=w._delete_session, retention_text="90 dias", parent=w)
    check("janela de Histórico mostra as aquisições", hd.table.rowCount() == 2 and "aquisições" in hd.info_label.text(),
          hd.info_label.text())
    hd.table.selectRow(1)  # a mais antiga (antes do Resetar limites)
    saved.clear()
    hd.report_btn.click()
    check("PDF de uma aquisição ANTIGA pelo Histórico", bool(saved) and "Aquisição de" in saved[0][1], f"{saved}")
    csv_path = os.path.join(TMP, "leituras.csv")
    mw.QFileDialog.getSaveFileName = staticmethod(lambda *a, **k: (csv_path, "CSV (*.csv)"))
    hd.export_btn.click()
    linhas = open(csv_path, encoding="utf-8-sig").read().splitlines()
    dados = [ln for ln in linhas if not ln.startswith("#")]
    n_primeira = w._history.count_readings(first_sid)
    check("CSV: cabeçalho, uma linha por leitura, ';' e vírgula decimal",
          dados[0] == "data_hora;t_s;inclinacao_graus;azimute_graus" and len(dados) == n_primeira + 1
          and dados[1].count(";") == 3 and "," in dados[1].split(";")[2] and "." not in dados[1].split(";")[2],
          f"{len(dados) - 1} linhas (esperado {n_primeira}); 1ª: {dados[1]}")
    check("CSV traz os dados da aquisição no topo", any(ln.startswith("# inicio:") for ln in linhas)
          and any("app: 2." in ln for ln in linhas))
    asked.clear()
    mw.QMessageBox.question = staticmethod(lambda _p, title, *a, **k: (asked.append(title), QMessageBox.Yes)[1])
    hd.delete_btn.click()
    check("apagar pede confirmação e remove a aquisição",
          "Apagar aquisição" in asked and w._history.get_session(first_sid) is None and hd.table.rowCount() == 1,
          f"{asked}, linhas {hd.table.rowCount()}")
    mw.QMessageBox.question = staticmethod(fake_question)
    hd.close()
    w._start()
    pump(1.0)
    check("aquisição em andamento não pode ser apagada",
          w._history.delete_session(w._history.current_session_id) is False)
    hd2 = HistoryDialog(w._history, lambda s: None, lambda s: None, lambda s: False, parent=w)
    check("Histórico marca a aquisição em andamento e desabilita Apagar",
          "em andamento" in hd2.table.item(0, 0).text() and not hd2.delete_btn.isEnabled())
    hd2.close()
    w._stop()

    # Retenção: padrão de 90 dias, opção em Configurações e limpeza ao trocar.
    from ui.settings_dialog import SettingsDialog
    check("retenção padrão: 90 dias", w._settings.history_retention_days == 90)
    dlg = SettingsDialog(w._settings, w, history_info=w._history_info())
    check("Configurações mostra a opção e o tamanho do histórico",
          dlg.retention_combo.currentData() == 90 and "leituras" in dlg.history_info_label.text(),
          dlg.history_info_label.text())
    w._history._conn.execute("UPDATE sessions SET started_at = started_at - ? WHERE id = ?", (40 * 86400.0, second_sid))
    w._history.flush()
    check("encurtar o prazo pede confirmação", w._confirm_retention_change(90, 30) is False and "Apagar histórico antigo" in asked,
          f"{asked}")
    check("aumentar o prazo não pede", w._confirm_retention_change(30, 365) is True)
    w._settings.history_retention_days = 30
    rem = w._prune_history()
    check("limpeza com 30 dias apaga a aquisição de 40 dias atrás",
          rem.get("sessions") == 1 and w._history.get_session(second_sid) is None, f"{rem}")
    check("mensagem da limpeza na barra", "removido" in w._prune_message(rem), w._prune_message(rem))
    w._settings.history_retention_days = 90

    # Resetar + Parar logo em seguida: a aquisição vazia não vira relatório vazio.
    w.acq_duration_combo.setCurrentIndex(0)
    w._start()
    pump(1.0)
    w._reset_limits()
    w._stop()
    saved.clear()
    w._generate_report()
    used = w._report_session()
    check("relatório ignora a aquisição vazia do fim", used is not None and w._history.count_readings(used.id) > 0)
    w.close()
except Exception as exc:  # noqa: BLE001
    import traceback

    traceback.print_exc()
    check("janela principal", False, repr(exc))

shutil.rmtree(TMP, ignore_errors=True)
print("RELATÓRIO: TUDO OK" if failures == 0 else "RELATÓRIO: HOUVE FALHA")
sys.exit(1 if failures else 0)
