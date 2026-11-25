@echo off
REM Instalador de dependencias do Inclinometro Avibras Aeroco (Windows).
REM Cria um ambiente virtual isolado em python-app\.venv e instala tudo
REM que esta em requirements.txt, sem alterar o Python global do sistema.

setlocal enabledelayedexpansion
cd /d "%~dp0.."

echo ============================================
echo  Inclinometro Avibras Aeroco - Instalador
echo ============================================
echo.

REM Python PORTATIL: se windows\python-portable\python.exe existir (copiado
REM do CD(2) DSF - distribuicao Python 3.10 "install_only" para Windows, sem
REM instalador), usa ele direto, sem exigir Python ja instalado no sistema
REM nem tocar no PATH da maquina. E o caminho normal em campo/fabrica.
set "PYTHON_EXE=python"
if exist "windows\python-portable\python.exe" (
    echo Usando o Python portatil do CD^(2^) DSF ^(windows\python-portable^)...
    set "PYTHON_EXE=%CD%\windows\python-portable\python.exe"
    goto :python_ok
)

where python >nul 2>nul
if errorlevel 1 (
    echo [ERRO] Python nao foi encontrado no PATH, e windows\python-portable
    echo tambem nao existe nesta pasta.
    echo.
    echo Copie a pasta "python" do CD^(2^) DSF para windows\python-portable,
    echo ou instale o Python 3.10 ou superior em:
    echo   https://www.python.org/downloads/
    echo.
    echo IMPORTANTE, se for instalar: na tela de instalacao do Python, marque
    echo a opcao "Add python.exe to PATH" antes de clicar em Install.
    echo.
    pause
    exit /b 1
)

:python_ok
echo Verificando versao do Python...
"%PYTHON_EXE%" --version

echo.
echo Criando ambiente virtual em ".venv"...
"%PYTHON_EXE%" -m venv .venv
if errorlevel 1 (
    echo [ERRO] Falha ao criar o ambiente virtual.
    pause
    exit /b 1
)

call .venv\Scripts\activate.bat

REM Instalacao OFFLINE: se windows\offline_packages existir (gerado por
REM windows\build_offline_bundle.bat numa maquina com internet, e copiado
REM junto com esta pasta para o DVD/CD do repositorio fisico da fabrica),
REM instala a partir dela, sem nenhuma tentativa de acesso a rede. Este e
REM o caminho normal de instalacao em campo/fabrica, sempre em Windows e
REM sempre sem internet.
if exist "windows\offline_packages" (
    echo.
    echo Pacote offline encontrado em windows\offline_packages.
    echo Instalando dependencias sem usar a internet...
    pip install --no-index --find-links=windows\offline_packages -r requirements.txt
    if errorlevel 1 (
        echo.
        echo [ERRO] Falha ao instalar a partir do pacote offline. Confirme que
        echo windows\offline_packages contem os .whl de TODOS os pacotes de
        echo requirements.txt ^(gerar novamente com build_offline_bundle.bat,
        echo numa maquina com internet, se a midia estiver incompleta^).
        pause
        exit /b 1
    )
    goto :instalado
)

echo.
echo Pacote offline nao encontrado ^(windows\offline_packages^) - instalando
echo pela internet. Este caminho e para desenvolvimento; a instalacao em
echo campo/fabrica deve usar a midia offline ^(ver build_offline_bundle.bat^).
echo.
echo Atualizando pip...
python -m pip install --upgrade pip

echo.
echo Instalando dependencias do software (isso pode levar alguns minutos)...
pip install -r requirements.txt -c windows\constraints.txt
if errorlevel 1 (
    echo.
    echo [ERRO] Falha ao instalar as dependencias. Verifique sua conexao
    echo com a internet ^(ou configuracoes de proxy^) e tente novamente.
    pause
    exit /b 1
)

:instalado

echo.
echo ============================================
echo  Instalacao concluida com sucesso!
echo.
echo  Para abrir o software, use: windows\run.bat
echo ============================================
pause
