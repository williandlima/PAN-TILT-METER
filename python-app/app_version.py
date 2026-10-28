"""Identidade desta versão do app desktop.

Existe para permitir manter **duas versões instaladas ao mesmo tempo** na
mesma máquina, para comparação:

- **versão 1** (branch `v1-inclinacao`, firmware 1.1.1): mede só a inclinação;
- **versão 2** (esta, na `main`): mede inclinação e azimute.

Como as duas se separam: cada uma tem seu próprio nome, sua própria pasta de
instalação, seu próprio `AppId` no Inno Setup (é ele que faz o Windows tratar
uma instalação como *upgrade* da outra, em vez de app separado) e sua própria
pasta de dados. Trocar de versão é uma questão de fazer checkout da branch
correspondente e reconstruir.

Manter em sincronia com `windows/Inclinometro.iss`, que precisa dos mesmos
valores mas não consegue importar Python.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

APP_VERSION = "2.2.0"

# Nome exibido na janela principal (título e cabeçalho da tela).
#
# Diferente de APP_VARIANT abaixo (que continua "2Eixos" — usado só para
# identificar arquivos/pastas internos e não deve mudar, sob risco de o app
# parar de achar o histórico já salvo) e do nome do instalador Windows
# (`windows/Inclinometro.iss`, MyAppName — ainda "Inclinometro 2 Eixos
# (Avibras Aeroco)", usado nos atalhos/Menu Iniciar/Painel de Controle),
# este é só o texto amigável mostrado dentro do próprio app.
APP_NAME = "Inclinômetro Avibras Aeroco"

# Sufixo usado em nomes de pasta/arquivo. Sem acento nem espaço de propósito.
APP_VARIANT = "2Eixos"


def user_data_dir() -> Path:
    """Pasta onde esta versão guarda dados que ela mesma escreve (o banco de
    histórico).

    Rodando a partir do código-fonte, fica ao lado do projeto — prático para
    desenvolvimento. Já instalado (executável do PyInstaller), o app mora em
    Program Files, onde um usuário **sem privilégio de administrador não pode
    escrever**: gravar o banco ali falharia. Então vai para a pasta de dados
    do próprio usuário, com o nome da variante no caminho — o que também
    mantém separados os históricos das duas versões instaladas lado a lado.
    """
    if getattr(sys, "frozen", False):
        base = os.environ.get("LOCALAPPDATA") or os.environ.get("APPDATA")
        root = Path(base) if base else Path.home() / ".local" / "share"
        directory = root / f"Inclinometro{APP_VARIANT}"
        directory.mkdir(parents=True, exist_ok=True)
        return directory
    return Path(__file__).resolve().parent
