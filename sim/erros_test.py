"""O app desktop não pode sumir sem dizer nada (app 2.4.3).

No PyQt5, uma exceção não tratada num clique aborta o processo; no .exe sem
console a janela simplesmente fechava (25/11/2026: "botão Iniciar fecha o
programa"). main._install_error_handlers() grava o erro em
%LOCALAPPDATA%/Inclinometro-2Eixos/erros.log, mostra uma mensagem e mantém o
programa aberto; erros de threads de fundo e falhas nativas também vão para o
log. Ver run.sh.
"""
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
APP = os.path.join(HERE, "..", "python-app")
failures = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global failures
    print(f"  {'OK    ' if ok else 'FALHOU'} {name}{': ' + detail if detail else ''}")
    if not ok:
        failures += 1


print("=== erros do app desktop: log em vez de fechar ===")
CHILD = r'''
import sys, threading
sys.path.insert(0, sys.argv[1])
import main as m
path = m._install_error_handlers()
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMessageBox, QPushButton
app = QApplication([])
QMessageBox.critical = staticmethod(lambda *a, **k: print("DIALOGO", a[2].splitlines()[0]))
b = QPushButton("x")
def boom():
    raise RuntimeError("falha de teste no clique")
b.clicked.connect(boom)
QTimer.singleShot(50, b.click)
QTimer.singleShot(100, lambda: threading.Thread(target=lambda: 1 / 0, name="leitura").start())
QTimer.singleShot(500, lambda: (print("VIVO"), app.quit()))
app.exec_()
print("LOG", path)
'''
with tempfile.TemporaryDirectory() as tmp:
    env = dict(os.environ, LOCALAPPDATA=tmp, QT_QPA_PLATFORM="offscreen")
    r = subprocess.run([sys.executable, "-c", CHILD, APP], env=env, capture_output=True, text=True, timeout=60)
    out = r.stdout
    check("exceção no clique não fecha o programa", r.returncode == 0 and "VIVO" in out, f"rc={r.returncode}")
    check("mensagem na tela com o erro", "DIALOGO RuntimeError: falha de teste no clique" in out)
    log = os.path.join(tmp, "Inclinometro-2Eixos", "erros.log")
    text = open(log, encoding="utf-8").read() if os.path.exists(log) else ""
    check("erro do clique gravado no log", "falha de teste no clique" in text)
    check("erro de thread de fundo gravado no log", "[thread leitura]" in text and "ZeroDivisionError" in text)

    native = r'''
import sys, threading, ctypes
sys.path.insert(0, sys.argv[1])
import main as m
m._install_error_handlers()
t = threading.Thread(target=lambda: ctypes.string_at(0), name="nativo"); t.start(); t.join()
'''
    r = subprocess.run([sys.executable, "-c", native, APP], env=env, capture_output=True, text=True, timeout=60)
    text = open(log, encoding="utf-8").read() if os.path.exists(log) else ""
    check("falha nativa deixa a pilha no log", r.returncode != 0 and "Fatal Python error" in text and "Thread" in text)

print("ERROS: TUDO OK" if failures == 0 else "ERROS: HOUVE FALHA")
sys.exit(1 if failures else 0)
