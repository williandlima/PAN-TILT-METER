@echo off
REM Prepara o pacote de instalacao OFFLINE do software desktop.
REM
REM Rodar esta etapa numa maquina COM internet (ex.: no ambiente de
REM desenvolvimento). Ela baixa todos os pacotes .whl exigidos por
REM requirements.txt para a pasta windows\offline_packages, ao lado
REM deste script.
REM
REM Depois de gerado, copiar a pasta python-app inteira (incluindo
REM windows\offline_packages) para o DVD/CD do repositorio fisico da
REM fabrica. install.bat detecta essa pasta automaticamente e instala
REM sem precisar de conexao a internet na maquina de destino.

setlocal
cd /d "%~dp0.."

where python >nul 2>nul
if errorlevel 1 (
    echo [ERRO] Python nao foi encontrado no PATH.
    echo Esta etapa precisa ser executada numa maquina com Python e
    echo conexao a internet, antes de gravar a midia de instalacao.
    pause
    exit /b 1
)

echo ============================================
echo  Preparando pacote de instalacao OFFLINE
echo ============================================
echo.
echo Baixando pacotes de requirements.txt para windows\offline_packages...
echo (isso precisa de conexao a internet - normal so nesta etapa)
echo.

python -m pip download -r requirements.txt -c windows\constraints.txt -d windows\offline_packages
if errorlevel 1 (
    echo.
    echo [ERRO] Falha ao baixar os pacotes. Verifique a conexao com a internet.
    pause
    exit /b 1
)

echo.
echo ============================================
echo  Pacote offline gerado com sucesso em:
echo    windows\offline_packages
echo.
echo  Isso e so a pasta offline_packages do CD(2) DSF. Antes de gravar
echo  o CD(2), baixar tambem (uma vez, nesta mesma maquina com internet)
echo  e copiar para windows\python-portable, na raiz do CD(2):
echo    - Distribuicao Python 3.10+ portatil, sem instalador
echo      (ex.: python-build-standalone, install_only, para Windows)
echo      - e' o runtime usado so para RODAR o software, nao um IDE.
echo.
echo  Depois: copiar a pasta python-app INTEIRA (com windows\offline_packages
echo  e windows\python-portable ja dentro) para o CD(2) DSF. Na maquina de
echo  destino (sem internet), windows\install.bat detecta essas pastas
echo  automaticamente e instala a partir delas.
echo.
echo  O CD(1) DAD e' outra midia, com o(s) instalador(es) do IDE (ex.:
echo  VSCodium ou VS Code) e um instalador completo do Python para
echo  Windows (python.org/downloads/windows) - ambos necessarios para
echo  desenvolver/editar o software, mas nao para so instalar/rodar
echo  ele. Nao tem codigo-fonte nem os arquivos do CD(2), e e'
echo  arquivado so como garantia/regra da fabrica, nunca usado na
echo  instalacao (ver ANEXO A do procedimento de instalacao, em docs/).
echo ============================================
pause
