# Inclinômetro — App Android (Kotlin)

App Android para leitura em tempo real dos **dois eixos** do inclinômetro
ESP32 — inclinação (tilt) e azimute (pan) — via BLE, com registro de limites
(mínimo/máximo) por eixo, calibração, Modo Vibração e geração de relatório em
PDF — equivalente ao software desktop em PyQt5, na mesma experiência, e ao
firmware em `firmware/`.

## Instalar as duas versões lado a lado

Dá para manter no mesmo aparelho a **versão 1** (mede só a inclinação) e a
**versão 2** (esta, que mede inclinação e azimute), para comparar. O que
separa as duas é o `applicationId`: o Android usa esse identificador para
decidir se um APK é um app novo ou uma atualização de outro, então dois APKs
com o mesmo id nunca coexistem — instalar um substitui o outro.

| | Versão 1 | Versão 2 |
|---|---|---|
| Nome no aparelho | Inclinômetro | Inclinômetro 2 Eixos |
| `applicationId` | `com.williandlima.inclinometro` | `com.williandlima.inclinometro.doiseixos` |
| `versionName` | 0.1.0 | 2.0.0 |
| Firmware correspondente | 1.1.1 | 1.3.1 |

Cada uma guarda seus próprios dados: o banco Room fica no armazenamento
privado de cada `applicationId`, então os históricos não se misturam e
desinstalar uma não apaga os da outra.

Para gerar o APK da **versão 2**, use o projeto como está hoje (branch
`main`). Para o da **versão 1**, mude para a branch dela:

```bash
git checkout v1-inclinacao
```

Guarde o APK gerado em outra pasta antes de voltar para a `main`. Naquela
branch o `applicationId` e o nome da versão 1 já estão no lugar, então o APK
sai com a identidade certa automaticamente — não é preciso editar nada.

> Use um app de cada vez contra o mesmo ESP32: feche a conexão de um antes
> de conectar o outro. A partir do firmware **1.3.1** o ESP32 volta a
> anunciar sozinho quando o cliente desconecta — em versões anteriores ele
> sumia do scan até ser resetado na mão, o que atrapalha justamente esse
> vaivém entre os dois apps.

## Requisitos

- Android Studio (Koala ou mais recente) com Android SDK 34 instalado.
- JDK 17.

## Abrir o projeto

Abra a pasta `android-app/` no Android Studio ("Open" → selecione esta
pasta). O Android Studio vai baixar as dependências (AndroidX, Compose,
Room) e configurar o SDK automaticamente na primeira sincronização.

> Este projeto foi criado fora do Android Studio (sem Android SDK disponível
> no ambiente de desenvolvimento usado), então **ainda não foi compilado
> com o Gradle/AGP real**. Como verificação parcial, o algoritmo de FFT
> (`limits/Fft.kt`) foi extraído e testado isoladamente com `kotlinc` puro
> (sem dependências Android) contra um sinal senoidal conhecido, confirmando
> que identifica a frequência dominante corretamente — o resto do código
> (Compose, BLE, Room) foi revisado manualmente, mas a primeira
> sincronização no Android Studio é o jeito confiável de confirmar que
> compila — abra o projeto lá antes de dar como validado.

## Uso

Na tela, escolha entre:
- **Simulação**: gera ângulos sintéticos (mesmos parâmetros do app desktop),
  para desenvolver/testar sem hardware.
- **Real (BLE)**: informe o endereço MAC do inclinômetro ESP32, ou toque em
  **Escanear** para listar os dispositivos anunciando o serviço BLE do
  inclinômetro nas proximidades e selecionar um da lista (`datasource/BleScanner.kt`,
  filtrado por `BleContract.SERVICE_UUID` — equivalente ao "Escanear" do app
  desktop). O app segue o contrato BLE definido em `datasource/BleContract.kt`,
  o mesmo implementado no firmware (`firmware/src/BleServer.cpp`).

O botão **"Testar conexão com ESP32"** (`datasource/BleConnectionTester.kt`)
faz uma leitura única do ângulo e da versão do firmware, sem iniciar uma
sessão de leitura contínua — mostra o resultado (verde) ou o erro específico
(vermelho), equivalente ao mesmo botão no app desktop.

O indicador de conexão mostra o estado (conectando/conectado/falha), a
exemplo do app desktop.

Com a leitura em execução, o botão **Calibrar** zera **os dois eixos** na
posição atual (envia comando ao ESP32 em modo real; aplica um deslocamento
equivalente em modo simulação). É uma ação só de propósito: o firmware zera
tilt e pan no mesmo comando.

## Os dois eixos

A tela mostra os dois eixos empilhados, cada um com seu valor em tempo real e
seu par de mínimo/máximo independente:

- **Inclinação (tilt)**, do acelerômetro, faixa -60° a +60°.
- **Azimute (pan)**, do giroscópio integrado com ZUPT no firmware (ver
  "Azimute (pan) pelo giroscópio" em `firmware/README.md`).

**Resolução de exibição** (app 2.2.0): abaixo do valor de cada eixo, o
seletor **Resolução: 0,25° / 0,1°** — também nas configurações de conexão.
0,25° é o padrão; 0,1° é para bancada, com referência de precisão. Cada eixo
tem o seu, a troca vale na hora (também durante a leitura) e a escolha fica
gravada no aparelho. Em 0,1° o valor aparece com uma casa decimal. A regra
(degrau + histerese de 20% do degrau) está em `ui/AngleDisplay.kt`, igual à do
app desktop (`python-app/ui/angle_display.py`); `sim/AngleDisplayTest.kt`
confere, com `kotlinc` puro, que os dois apps mostram exatamente os mesmos
valores. Só muda o arredondamento da tela — histórico e relatório usam sempre
o ângulo bruto.

Compatibilidade: um ESP32 com firmware anterior à v1.2.0 não tem a
characteristic de azimute. O app detecta isso sozinho e segue funcionando só
com a inclinação — o painel de azimute mostra `--.--°` com a nota "firmware
sem este eixo", e o eixo fica de fora do histórico e do relatório em vez de
registrar zeros falsos.

No modo simulação os dois eixos têm comportamentos diferentes de propósito,
imitando o real: o tilt oscila continuamente com ruído, e o pan fica parado a
maior parte do tempo e se desloca em rajadas — que é justamente o padrão de
uso que torna a medição por giroscópio confiável.

O botão **Modo Vibração** faz uma captura em alta taxa configurável
(duração + taxa de amostragem) **nos dois eixos**, útil para caracterizar
variação angular por vento/vibração — ao final, mostra desvio padrão/RMS/
pico-a-pico e a **frequência dominante** de cada eixo (amplitude + SNR, com
detecção sub-bin por interpolação parabólica; "nenhum pico confiável" se o
sinal for compatível com ruído), e permite salvar um relatório em PDF com
uma página por eixo (gráfico no tempo + espectro de frequência, com o pico
marcado). Cada captura fica salva separada das sessões de monitoramento
contínuo no histórico. Pipeline da FFT (`limits/Fft.kt`) equivalente ao do
app desktop (`limits/vibration_stats.py`): tendência linear removida, janela
de Hann, amplitude corrigida e SNR adaptativo ao número de bins.

O eixo de azimute chega do firmware como **velocidade angular** (°/s), não
como ângulo — o ZUPT que produz o ângulo de pan cancela de propósito o que
integra enquanto o eixo está parado, que é justamente a condição de um ensaio
de vibração. O app integra e remove a tendência linear
(`datasource/VibrationReadings.kt`), e roda a detecção do pico no espectro da
taxa, onde o ruído do giroscópio é branco. Detalhes em "Modo Vibração no eixo
de pan" no `firmware/README.md`.

O modo real (BLE) segue o mesmo contrato implementado no firmware do ESP32
(`firmware/`), mas ainda não foi validado contra hardware físico.

## Identidade visual (Avibras Aeroco)

A interface usa a paleta azul marinho + laranja da Avibras Aeroco
(`ui/theme/Color.kt`), mesma identidade do app desktop. Para exibir uma
logo no cabeçalho, adicione um recurso `res/drawable/logo.png` (ou
`.xml`/vetor) — se não existir, o app mostra só o título em texto com as
mesmas cores (checagem em tempo de execução, via
`resources.getIdentifier`).

## Estrutura

```
datasource/    fontes de dados de ângulo (simulada e BLE real) + contrato BLE
limits/        rastreamento de limites (mín/máx), histórico persistente (Room/SQLite),
               estatísticas/FFT de vibração
ui/            ViewModel + tela principal (Jetpack Compose)
report/        geração de relatório em PDF nativo (PdfDocument), sem dependência externa
```
