# Instalação no Windows

Este pacote instala todas as bibliotecas e dependências do Inclinômetro
Avibras Aeroco em qualquer computador Windows 10/11 (64 bits), sem mexer
na instalação global do Python da máquina (tudo fica isolado em um
ambiente virtual dentro da própria pasta `python-app`).

Existem três caminhos, dependendo de quem vai usar o resultado:

> **Para o operador final do software (sem conhecimento de programação),
> use sempre a Opção 3 (instalador Setup.exe).** É a única que cria um
> ícone na Área de Trabalho e no Menu Iniciar, com a logo da Avibras
> Aeroco — o software abre por duplo clique, como qualquer programa do
> Windows, sem terminal, sem arquivo `.bat` e sem editor de código. As
> Opções 1 e 2 abaixo são só para quem desenvolve/mantém o software.

## Opção 1 — Instalar e rodar a partir do código-fonte (uso: desenvolvimento)

Requisito: **Python 3.10 ou superior** instalado no computador.
Se não tiver, baixe em https://www.python.org/downloads/ e, na tela de
instalação, marque a opção **"Add python.exe to PATH"**.

Passos:

1. Copie a pasta `python-app` inteira para o computador Windows.
2. Dê duplo clique em `windows\install.bat`.
   - Cria um ambiente virtual em `python-app\.venv`.
   - Baixa e instala automaticamente: PyQt5, pymodbus, pyserial, bleak,
     reportlab e matplotlib (todos listados em `requirements.txt`).
3. Para abrir o software (agora e nas próximas vezes), dê duplo clique
   em `windows\run.bat`.

Não é necessário rodar `install.bat` de novo, a menos que o
`requirements.txt` mude (nesse caso, rode novamente para atualizar as
dependências).

### Instalação sem internet (uso normal em campo/fábrica)

A instalação em campo/fábrica é sempre feita em Windows e **sem conexão à
internet** — a mídia (DVD/CD) vem pronta do repositório físico da fábrica.
`install.bat` detecta isso automaticamente: se existir a pasta
`windows\offline_packages` (com os `.whl` de todas as dependências), ele
instala a partir dela, sem tentar acessar a rede.

Essa pasta é preparada **uma vez**, numa máquina com internet (normalmente
no ambiente de desenvolvimento), antes de gravar a mídia:

1. Numa máquina com Python e internet, dê duplo clique em
   `windows\build_offline_bundle.bat`. Ele baixa todos os `.whl` de
   `requirements.txt` para `windows\offline_packages`.
2. Grave a pasta `python-app` inteira (já incluindo `windows\offline_packages`)
   no DVD/CD, e arquive no repositório físico da fábrica.
3. Na máquina de destino (Windows, sem internet), copie a pasta do
   DVD/CD e rode `windows\install.bat` normalmente — ele reconhece a
   pasta offline e instala sem rede.

Se `windows\offline_packages` não existir, `install.bat` cai de volta na
instalação pela internet (útil em desenvolvimento, mas não é o caminho de
uso em campo).

## Opção 2 — Gerar um executável autônomo (uso: desenvolvimento/manutenção — sem ícone de atalho)

Útil para instalar em computadores onde não se quer/pode instalar Python
(ex: máquina de produção/chão de fábrica). Gera uma pasta com um
`Inclinometro2Eixos.exe` que roda sozinho.

**Importante:** o executável precisa ser **gerado em um computador
Windows** (o PyInstaller não faz cross-compilação de Linux/macOS para
`.exe`). O passo a passo abaixo é feito uma vez, em qualquer PC Windows
com Python instalado; o resultado (pasta `dist\Inclinometro2Eixos`) pode então
ser copiado para quantos computadores forem necessários, mesmo sem
Python.

Passos (em um Windows com Python instalado):

1. Rode `windows\install.bat` (Opção 1, passo 2) se ainda não rodou.
2. Dê duplo clique em `windows\build_exe.bat`.
   - Instala o PyInstaller e empacota o app.
   - Ao final, gera a pasta `dist\Inclinometro2Eixos\`, contendo
     `Inclinometro2Eixos.exe` e todos os arquivos necessários.
3. Copie a pasta `dist\Inclinometro2Eixos` inteira para o computador de
   destino (pendrive, rede, etc.) e rode `Inclinometro2Eixos.exe` diretamente
   — não precisa instalar nada nesse computador.

## Opção 3 — Gerar um instalador (Setup.exe) com atalhos e desinstalador (USE ESTA para o operador final)

A forma mais próxima de um "programa instalado de verdade": um único
arquivo `Inclinometro-2Eixos-Setup-2.0.0.exe` que, ao ser executado no computador
de destino, instala o programa em `Arquivos de Programas`, cria atalho no
Menu Iniciar (e, opcionalmente, na Área de Trabalho) e registra um
desinstalador em "Adicionar ou remover programas" — sem precisar de Python
em nenhum dos dois computadores (o de geração nem o de destino).

**Requisito extra** (só na máquina onde o instalador é **gerado**, uma
única vez): instalar o [Inno Setup](https://jrsoftware.org/isdl.php)
(gratuito) — instalação padrão, sem opções especiais.

Passos (em um Windows com Python **e** Inno Setup instalados):

1. Rode `windows\install.bat` (Opção 1, passo 2) se ainda não rodou.
2. Dê duplo clique em `windows\build_installer.bat`.
   - Gera o executável autônomo (mesmo processo da Opção 2).
   - Compila o instalador com o Inno Setup.
   - Ao final, gera `windows\installer_output\Inclinometro-2Eixos-Setup-2.0.0.exe`.
3. Copie esse único arquivo `.exe` para o(s) computador(es) de destino e
   execute — o assistente de instalação cuida do resto. Não precisa
   instalar Python nem Inno Setup nesses computadores.

Esse arquivo é o mais indicado para distribuir para os usuários finais do
software (ex: equipe de chão de fábrica); as Opções 1 e 2 continuam úteis
para desenvolvimento/testes.

## Instalar as duas versões lado a lado

Dá para manter no mesmo computador a **versão 1** (mede só a inclinação) e a
**versão 2** (mede inclinação e azimute), para comparar. Elas não se
atropelam porque a versão 2 tem nome, pasta de instalação, `AppId` do Inno
Setup e pasta de dados próprios — o `AppId` é o que importa de verdade: é
por ele que o Windows decide se uma instalação é um app novo ou um
*upgrade* do outro.

| | Versão 1 | Versão 2 |
|---|---|---|
| Nome no Menu Iniciar | Inclinometro Avibras Aeroco | Inclinometro 2 Eixos (Avibras Aeroco) |
| Pasta de instalação | `Arquivos de Programas\Inclinometro` | `Arquivos de Programas\Inclinometro2Eixos` |
| Instalador gerado | `Inclinometro-Setup-1.0.0.exe` | `Inclinometro-2Eixos-Setup-2.0.0.exe` |
| Firmware correspondente | 1.1.1 | 1.3.1 |

Para gerar o instalador da **versão 2**, siga a Opção 3 acima com o código
como está hoje.

Para gerar o da **versão 1**, mude para a branch dela e repita a Opção 3:

```bat
git checkout v1-inclinacao
windows\build_installer.bat
```

Guarde o `.exe` gerado em outra pasta antes de voltar para a `main`, porque
`installer_output` é reaproveitado entre as duas gerações. Naquela branch os
identificadores da versão 1 já estão no lugar, então o instalador sai com a
identidade certa automaticamente — não é preciso editar nada.

**Cada versão tem seu próprio histórico.** O banco de dados fica em
`%LOCALAPPDATA%\Inclinometro<variante>\`, com uma pasta por versão, então os
dados de uma não aparecem na outra e desinstalar uma não apaga os da outra.

## Observações

- **Antivírus/SmartScreen:** executáveis gerados com PyInstaller às
  vezes disparam um alerta de "aplicativo desconhecido" na primeira
  execução (falso positivo comum, por não terem assinatura digital). Se
  isso acontecer, use "Mais informações → Executar assim mesmo" ou
  adicione uma exceção no antivírus. Assinar digitalmente o `.exe` é uma
  opção futura se isso incomodar no ambiente corporativo.
- **Logo/ícone:** a logo da Avibras Aeroco já está versionada em
  `assets/logo.png` (empacotada automaticamente ao gerar o executável — ver
  seção "Identidade visual" no `README.md` principal) e um ícone derivado
  dela em `assets/logo.ico` é usado no `.exe`, no instalador e nos atalhos
  gerados.
- **Bluetooth (BLE):** o modo de leitura via Bluetooth usa o adaptador
  Bluetooth nativo do próprio computador (via `bleak`); não precisa de
  dongle extra, mas o computador precisa ter Bluetooth.
- **USB/Modbus:** o modo via cabo USB direto ao ESP32 pode precisar do
  driver do chip USB-serial da placa (CP2102N, CH340 ou FTDI, dependendo do
  modelo). **A placa usada no projeto (ESP32-DevKitC V4) tem um chip
  CP2102N (Silicon Labs)** — o driver "CP210x VCP" normalmente já vem com o
  Windows 10/11; se a porta COM não aparecer sozinha no Gerenciador de
  Dispositivos ao conectar, baixe e instale o driver oficial da Silicon
  Labs. Para distâncias maiores que ~5m, use um cabo de extensão USB ativo.

## Solução de problemas

| Problema | Causa provável | Solução |
|---|---|---|
| `'python' não é reconhecido como um comando...` | Python não está no PATH | Reinstale o Python marcando "Add python.exe to PATH" |
| Falha ao instalar a partir do pacote offline (`windows\offline_packages`) | Mídia gravada com pacotes incompletos/desatualizados em relação a `requirements.txt` | Gerar novamente `windows\offline_packages` com `build_offline_bundle.bat`, numa máquina com internet, e regravar a mídia |
| Falha ao instalar dependências pela internet (Opção 1, sem mídia offline) | Sem internet ou proxy corporativo bloqueando | Verifique a conexão; em rede corporativa, configure o proxy do `pip` ou peça liberação de acesso ao PyPI — ou use a instalação offline acima |
| Executável não abre / fecha sozinho | Antivírus bloqueou ou faltou gerar em máquina Windows | Veja "Antivírus/SmartScreen" acima; gere o `.exe` novamente em um Windows |
