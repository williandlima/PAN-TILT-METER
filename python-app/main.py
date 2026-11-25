"""Ponto de entrada do painel desktop do inclinômetro (PyQt5)."""
import os
import sys

# Garante que a raiz de python-app/ esteja no sys.path mesmo quando o
# interpretador é iniciado de outro diretório de trabalho ou por uma
# ferramenta (VSCode, atalho, .exe empacotado) que não faz isso por conta
# própria — os módulos do projeto (ui, data_source, limits, report) são
# importados de forma absoluta a partir desta pasta.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import datetime  # noqa: E402
import faulthandler  # noqa: E402
import threading  # noqa: E402
import traceback  # noqa: E402

from PyQt5.QtCore import Qt  # noqa: E402
from PyQt5.QtWidgets import QApplication, QMessageBox  # noqa: E402

from ui.main_window import MainWindow  # noqa: E402

# Sem isso, num notebook com escala de tela >100% (muito comum em painéis
# industriais com monitor 4K/QHD) o Qt desenha os widgets no tamanho físico
# em pixels, ignorando a escala do sistema — telas e botões saem menores (e
# desproporcionais entre monitores diferentes) do que o usuário vê no resto
# do sistema operacional. Precisa ser definido antes de criar o QApplication.
QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)


def _log_path() -> str:
    """Arquivo de erros do app, numa pasta do usuário que sempre aceita
    gravação (no app instalado, a pasta do programa não aceita)."""
    base = os.environ.get("LOCALAPPDATA") or os.path.expanduser("~")
    folder = os.path.join(base, "Inclinometro-2Eixos")
    try:
        os.makedirs(folder, exist_ok=True)
    except OSError:
        folder = os.path.expanduser("~")
    return os.path.join(folder, "erros.log")


def _install_error_handlers() -> str:
    """O programa não pode sumir sem dizer nada (app 2.4.3).

    No PyQt5, uma exceção não tratada dentro de um clique (slot) ABORTA o
    processo — no .exe sem console, a janela simplesmente fecha. Agora:
    - exceção na thread da tela: vai para o log e aparece numa mensagem, e o
      programa continua aberto;
    - exceção numa thread de fundo (leitura BLE/USB): vai para o log;
    - falha nativa (biblioteca em C, ex.: o Bluetooth do Windows): o
      faulthandler grava no log a pilha de todas as threads antes de o
      processo morrer — é a única pista nesse caso.
    """
    path = _log_path()

    def write(text: str) -> None:
        try:
            with open(path, "a", encoding="utf-8") as fh:
                fh.write(f"\n=== {datetime.datetime.now():%d/%m/%Y %H:%M:%S} ===\n{text}\n")
        except OSError:
            pass

    def excepthook(exc_type, exc, tb) -> None:
        text = "".join(traceback.format_exception(exc_type, exc, tb))
        write(text)
        sys.__stderr__ and sys.__stderr__.write(text)
        if QApplication.instance() is not None:
            QMessageBox.critical(
                None,
                "Erro inesperado",
                f"{exc_type.__name__}: {exc}\n\nO programa continua aberto. Detalhes gravados em:\n{path}",
            )

    def thread_excepthook(args) -> None:
        write(f"[thread {args.thread.name if args.thread else '?'}]\n"
              + "".join(traceback.format_exception(args.exc_type, args.exc_value, args.exc_traceback)))

    sys.excepthook = excepthook
    threading.excepthook = thread_excepthook
    try:
        # Mantido aberto durante toda a execução (o faulthandler escreve nele
        # no momento da falha, sem poder abrir arquivo).
        global _fault_file
        _fault_file = open(path, "a", encoding="utf-8")
        faulthandler.enable(file=_fault_file, all_threads=True)
    except OSError:
        pass
    return path


_fault_file = None


def main() -> None:
    _install_error_handlers()
    app = QApplication(sys.argv)
    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
