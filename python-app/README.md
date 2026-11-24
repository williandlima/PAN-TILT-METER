# Inclinômetro — Software Desktop (PyQt5)

Software desktop para leitura em tempo real dos **dois eixos** do inclinômetro
ESP32 — inclinação (tilt) e azimute (pan) — via Modbus RTU (cabo USB direto)
ou Bluetooth Low Energy (BLE), com registro de limites (mínimo/máximo) por
eixo e geração de relatório em PDF.

## Instalação

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**Windows:** use o pacote de instalação pronto em
[`windows/`](windows/INSTALACAO_WINDOWS.md) — instala tudo com um duplo
clique (`windows\install.bat`) e também permite gerar um executável
autônomo, ou um instalador completo (`Setup.exe` com atalhos e
desinstalador) via `windows\build_installer.bat`, que roda em qualquer
computador Windows sem precisar instalar Python.

## Uso

```bash
python main.py
```

**No VSCode**: abra a pasta raiz do repositório (`Inclinometer_Esp32`) e use
**F5** (Run and Debug) — já existe uma configuração pronta em
`.vscode/launch.json` que sempre executa `python-app/main.py`, independente
de qual arquivo esteja aberto no editor. **Não use o botão ▶ "Run Python
File"** enquanto estiver editando um arquivo dentro de `ui/`, `data_source/`,
`limits/` ou `report/` — esse botão roda o arquivo em foco, não o `main.py`,
e como esses módulos usam import absoluto a partir de `python-app/`
(`from data_source... `, `from ui...`), rodar um deles isoladamente falha com
erro de import (ex.: `No module named 'app_version'`). O `main.py` sempre
funciona porque insere a própria pasta no `sys.path` antes de importar
qualquer módulo do projeto.

Na tela, use **Configurações** para escolher entre:
- **Modo Simulação**: gera ângulos sintéticos (oscilação suave em torno de 60°),
  útil para desenvolver/testar sem o hardware pronto.
- **Real (USB/Modbus RTU)**: conecta via cabo USB direto ao ESP32 (escolha a
  porta serial que o ESP32 aparecer, baud rate e endereço do escravo Modbus).
  Para distâncias maiores que ~5m, use um cabo de extensão USB ativo.
- **Real (Bluetooth BLE)**: conecta via Bluetooth do próprio notebook — use
  **Escanear** para listar dispositivos próximos ou informe o endereço
  manualmente. Requer a biblioteca `bleak` (já incluída no `requirements.txt`),
  que usa o adaptador Bluetooth nativo do computador (sem dongle extra).

Em qualquer um dos modos reais, o botão **"Testar conexão com ESP32"** faz uma
leitura única para confirmar a comunicação antes de iniciar uma sessão —
mostra o ângulo lido e a versão do firmware conectado (verde), ou o erro
específico (vermelho).

O firmware do ESP32 já implementa os dois modos reais (`firmware/`), mas
ainda não foi validado contra hardware físico: USB/Modbus RTU lê o
registrador de entrada Modbus configurado em `data_source/modbus_source.py`;
BLE segue o contrato de serviço/característica documentado em
`data_source/ble_source.py` (o mesmo usado pelo app Android, para os dois
transportes ficarem consistentes).

Também é possível **calibrar** (zerar os dois eixos na posição atual) pelo
botão **Calibrar**, disponível com a leitura em execução — em modo real (USB
ou BLE), envia um comando ao ESP32; em modo simulação, aplica um deslocamento
equivalente aos dados sintéticos. É uma ação só de propósito: o firmware zera
tilt e pan no mesmo comando.

## Os dois eixos

A tela mostra os dois eixos lado a lado, cada um com seu valor em tempo real e
seu par de mínimo/máximo independente:

- **Inclinação (tilt)**, do acelerômetro, faixa -60° a +60°.
- **Azimute (pan)**, do giroscópio integrado com ZUPT no firmware (ver
  "Azimute (pan) pelo giroscópio" em `firmware/README.md`).

Compatibilidade: um ESP32 com firmware anterior à v1.2.0 não mede azimute. O
app detecta isso sozinho e segue funcionando só com a inclinação — o painel de
azimute mostra `--.--°` com a nota "firmware sem este eixo", e o eixo fica de
fora do histórico e do relatório em vez de registrar zeros falsos. No modo
USB/Modbus isso é detectado pela exceção de endereço inválido que o firmware
antigo devolve; no BLE, pela ausência da characteristic de pan.

No modo simulação os dois eixos têm comportamentos diferentes de propósito,
imitando o real: o tilt oscila continuamente com ruído, e o pan fica parado a
maior parte do tempo e se desloca em rajadas — que é justamente o padrão de
uso que torna a medição por giroscópio confiável.

O indicador abaixo do modo mostra o estado da conexão com o ESP32, seja via
USB ou BLE:

- **◐ Conectando...** — de "Iniciar" até a primeira leitura (a barra de
  status diz "Conectando a ...", nunca "Conectado" antes de chegar dado);
- **● Conectado** (verde) — só enquanto chegam leituras da fonte em
  execução;
- **◐ Sem dados do ESP32** (laranja) — mais de 2 s sem leitura, antes de a
  fonte declarar a queda (no BLE, a queda pode levar 5 s ou mais para ser
  detectada; até lá o verde ficava aceso sem dado nenhum);
- **● Falha de conexão** (vermelho) — erro ou queda reportada pela fonte;
  volta ao verde sozinho quando a reconexão automática traz leituras.

Leituras e erros que ainda estavam na fila quando se clica em "Parar" (ou de
uma fonte anterior, após reiniciar) são descartados: antes eles acendiam o
verde com a leitura parada. Testado em `sim/conexao_test.py`.

**Resolução de exibição** (app 2.2.0): cada cartão de eixo tem um seletor
**Resolução: 0,25° / 0,1°** logo abaixo do valor — o mesmo de Configurações
("Resolução da inclinação" / "Resolução do azimute"). 0,25° é o padrão (não
tremula); 0,1° é para bancada, comparando com uma referência de precisão
(ex: nível eletrônico Mitutoyo, mesa giratória). Cada eixo tem o seu, a troca
vale na hora e a escolha é lembrada na próxima vez que o app abrir (arquivo
`Avibras\Inclinometro.ini` na pasta de configurações do usuário). Em 0,1° o
valor aparece com uma casa decimal (`12.3°`). A histerese que impede a tela de
alternar entre dois degraus é proporcional ao degrau (20%): a tela fica no
máximo 0,07° atrás da leitura em 0,1° (0,175° em 0,25°). Não muda a resolução
do sensor nem do dado gravado no histórico/relatório (ambos já são de 0,01°) —
só o arredondamento mostrado na tela. Regra em `ui/angle_display.py`, a mesma
do app Android; testada em `sim/exibicao_test.py` (ver `sim/run.sh`).

## Aquisição para o relatório PDF (app 2.3.0)

O botão **"Gerar relatório PDF"** usa as leituras de **uma aquisição**:

- **Começa** no **Iniciar** — ou em **Resetar limites**/**Calibrar** com a
  leitura em andamento, que abrem uma aquisição nova (os mín/máx de antes do
  reset não valem mais; a aquisição anterior continua no histórico).
- **Termina** no **Parar**, ou sozinha ao fim do **tempo programado**.
- Gerado com a leitura em andamento, o relatório usa os dados até o momento;
  com a leitura parada, usa a última aquisição que tem leituras.

Acima dos botões, a linha **"Aquisição para relatório"** escolhe o tempo —
**Livre (até "Parar")** (padrão), 1, 5, 10, 30 min ou 1 h — e, durante a
leitura, mostra o cronômetro (`gravando 03:12 / 10:00 · 768 leituras`). Com
um tempo definido, a leitura para sozinha ao fim dele e o app oferece gerar o
relatório. Parada, a linha mostra a última aquisição (início, duração,
leituras).

Cada leitura é o valor da tela, recebido a ~4-5 Hz (poll Modbus de 250 ms,
notify BLE de ~200 ms); os mín/máx vêm do próprio ESP32, medidos a 100 Hz,
com firmware v1.5.0+. O PDF abre com **Resultado** (mín/máx e duração) e
**Aquisição dos dados**: início, fim, duração, tempo programado (concluída /
interrompida / em andamento), leituras de cada eixo, taxa média e intervalo
típico, interrupções (> 2 s sem leitura, ex: queda do Bluetooth — em laranja
nos gráficos, fora da curva), cobertura e de onde vieram os mín/máx (também
por evento, na coluna **"Medido por"**). O eixo de tempo dos gráficos conta a
partir do início da aquisição, em s, min ou h conforme a duração. Cálculo em
`report/acquisition.py`; testes em `sim/relatorio_test.py`.

## Modo Vibração: CSV das amostras (app 2.4.1)

Na janela de resultado da captura, **Salvar CSV...** grava uma linha por
amostra (separador `;`, vírgula decimal, abre direto no Excel): tempo, ângulo
de tilt (acelerômetro, sem filtro de movimento), **taxa crua do pan (°/s)** e
a integral pura dela, sem remover tendência. O relatório PDF continua com o
pan sem tendência (bom para ver oscilação), mas isso transforma um degrau em
duas rampas; para diagnosticar trancos do motor do PTU (o pan andando na
partida e na parada do tilt), use o CSV. A janela não fecha ao salvar o CSV:
dá para salvar o PDF em seguida.

## Histórico: prazo de retenção (app 2.4.0)

O app grava cada leitura (~4 por segundo) num banco SQLite na pasta de dados
do usuário — 8 h de leitura são ~115 mil linhas (~4 MB) — e até a 2.3.0 nada
era apagado. Agora:

- **Configurações → "Manter histórico por"**: 30, 90 (padrão), 180 dias,
  1 ano ou Sempre. Abaixo da opção aparece o tamanho atual do histórico.
- A limpeza roda **ao abrir o app** e ao mudar o prazo: apaga as aquisições
  (leituras e eventos de limite) e as capturas de vibração mais antigas que
  o prazo, remove aquisições encerradas sem nenhuma leitura e compacta o
  arquivo. A aquisição em andamento nunca é apagada. O que foi removido
  aparece na barra de status; encurtar o prazo pede confirmação.
- **Os PDFs já gerados não são afetados** — só não dá mais para gerar o
  relatório de uma aquisição apagada. Para guardar tudo, escolha "Sempre".
- As leituras são gravadas em lote (no máximo uma gravação em disco por
  segundo, em vez de uma por leitura).
- Cada aquisição guarda a **versão do app e do firmware**, que aparece no
  relatório PDF ("Versões (app / firmware)").

**Tela de Histórico** (botão "Histórico..." ao lado da aquisição): lista
todas as aquisições com leituras (início, duração, modo, leituras, mín/máx
das leituras de cada eixo, tempo programado e firmware) e permite, para a
selecionada, **gerar o relatório PDF** (duplo clique também), **exportar as
leituras em CSV** (`;` e vírgula decimal, abre direto no Excel; dados da
aquisição e eventos de limite em linhas `#` no topo) ou **apagar** (com
confirmação; a aquisição em andamento não pode ser apagada). Antes, o
relatório só podia ser gerado da aquisição atual ou da última.

Código em `limits/history_store.py` (`prune()`, `list_session_summaries()`),
`limits/history_export.py` e `ui/history_dialog.py`; testes em
`sim/relatorio_test.py`.

## Modo Vibração

A leitura contínua normal (poll a cada ~250ms) é boa para acompanhar o
ângulo, mas lenta demais para caracterizar **vibração** — por exemplo, medir
a variação angular de um pan-tilt sob efeito de vento com o veículo parado.
Para isso existe o botão **"Modo Vibração"**, disponível com a leitura em
execução (habilite calibrando a posição de referência antes, com o botão
**Calibrar**, para que a variação registrada seja relativa a ela):

1. Escolha a **duração** (padrão 30s) e a **taxa de amostragem** (padrão
   50Hz) da captura.
2. O app mostra o progresso enquanto a captura acontece (pode ser cancelada).
3. Ao final, mostra um resumo estatístico **de cada eixo**: desvio padrão,
   RMS, pico a pico, mínimo, máximo e a frequência dominante (com amplitude
   e SNR — ou "nenhum pico confiável" se o sinal for compatível com ruído).
4. Opcionalmente, gera um **relatório em PDF** próprio, com uma página por
   eixo: gráfico da variação angular no tempo e **espectro de frequência
   (FFT)** — útil para identificar uma eventual frequência de ressonância
   dominante (ex: balanço do mastro sob vento), com a frequência dominante
   marcada no gráfico.

O eixo de azimute chega do firmware como **velocidade angular** (°/s), não
como ângulo: o ângulo de pan é obtido por integração com ZUPT, e o ZUPT
cancela de propósito o que integra enquanto o eixo está parado — que é
exatamente a condição de um ensaio de vibração. O app integra e remove a
tendência linear (que é o que o bias do giroscópio vira ao ser integrado).
Detalhes em "Modo Vibração no eixo de pan" no `firmware/README.md`.

A análise espectral (`limits/vibration_stats.py`) segue um pipeline padrão
de processamento de sinais para o resultado ser confiável e fácil de ler:
remoção de tendência linear, janela de Hann (reduz vazamento espectral),
amplitude de um lado só corrigida (valor em graus bate com a amplitude
física real da oscilação) e detecção do pico dominante com interpolação
parabólica (sub-bin) e um limiar de SNR que se ajusta ao número de bins do
espectro (evita falso positivo em sinais só com ruído). O app Android usa
exatamente o mesmo pipeline (`limits/Fft.kt`).

No eixo de azimute a detecção do pico roda no espectro da **taxa**, e só a
amplitude encontrada é convertida para graus. O motivo é que o limiar de SNR
supõe piso de ruído plano: o ruído do giroscópio é branco na taxa, mas no
ângulo integrado vira passeio aleatório (espectro caindo com 1/f²), e nessa
forma ruído puro era apontado como "frequência dominante" em 100% dos testes.

Cada captura é salva separada das sessões de monitoramento contínuo no
histórico (`limits/history_store.py`), para não misturar os dois tipos de
dado. Em modo simulação, a captura gera uma vibração sintética (duas
oscilações de baixa amplitude + ruído) só para exercitar todo o fluxo sem
hardware — em modo real (USB ou BLE), segue o contrato implementado no
firmware, documentado nos módulos `data_source/modbus_source.py` e
`data_source/ble_source.py` (ainda não confirmado/testado com hardware real).

## Identidade visual (Avibras Aeroco)

A interface usa a paleta azul marinho + laranja da Avibras Aeroco
(`ui/main_window.py`, constantes `NAVY`/`ORANGE`). A logo oficial já está
versionada em `assets/logo.jpg` e aparece automaticamente no cabeçalho —
para trocar por um arquivo diferente, basta substituir por `assets/logo.png`,
`assets/logo.jpg` ou `assets/logo.jpeg` (o app procura nessa ordem); sem
nenhum desses, mostra só o título em texto com as mesmas cores.

## Estrutura

```
assets/        logo (assets/logo.jpg)
data_source/   fontes de dados de ângulo (simulada, Modbus RTU via USB e BLE real)
limits/        rastreamento de limites (mín/máx) e histórico persistente (SQLite)
ui/            janela principal e diálogo de configurações (PyQt5)
report/        geração de relatório em PDF a partir do histórico
```
