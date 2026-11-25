@echo off
REM Gera um executavel Windows autonomo do Inclinometro (nao precisa de
REM Python instalado no computador de destino). Rode windows\install.bat
REM primeiro. Precisa ser executado em uma maquina Windows.

cd /d "%~dp0.."

if not exist ".venv\Scripts\activate.bat" (
    echo [ERRO] Ambiente virtual nao encontrado.
    echo Execute primeiro "windows\install.bat".
    pause
    exit /b 1
)

call .venv\Scripts\activate.bat

echo Instalando PyInstaller...
pip install -r windows\requirements-build.txt -c windows\constraints.txt
if errorlevel 1 (
    pause
    exit /b 1
)

echo.
echo Gerando executavel autonomo (isso pode levar alguns minutos)...
pyinstaller --noconfirm windows\Inclinometro.spec
if errorlevel 1 (
    pause
    exit /b 1
)

echo.
echo ============================================
echo  Executavel gerado em: dist\Inclinometro2Eixos\Inclinometro2Eixos.exe
echo.
echo  A pasta "dist\Inclinometro2Eixos" inteira pode ser copiada
echo  para qualquer computador Windows (10/11, 64 bits),
echo  mesmo sem Python instalado.
echo ============================================
pause
