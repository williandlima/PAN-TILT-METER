# docs

Documentação do projeto do inclinômetro/azimutímetro eletrônico ESP32,
reunida numa única pasta: os entregáveis formais em padrão ABNT e a
documentação técnica de referência mantida junto ao código.

## Entregáveis formais (padrão ABNT — NBR 14724)

Relatório e lista de materiais, com capa, folha de rosto, histórico de
revisões, sumário automático, margens 3-2-3-2cm, fonte Times New Roman 12pt,
espaçamento 1,5.

| Documento | Rev. | Conteúdo |
|---|---|---|
| [relatorio-tecnico-inclinometro.docx](relatorio-tecnico-inclinometro.docx) | 03 | Relatório técnico do projeto: fundamentação teórica, arquitetura do sistema, cálculos e modelagem matemática, protocolos de comunicação, modos de operação e resultados/validação. |
| [lista-de-materiais-hardware.docx](lista-de-materiais-hardware.docx) | 00 | Lista de materiais (BOM): componentes, especificações técnicas, datasheets dos fabricantes, ligações elétricas do sensor e opções de alimentação em avaliação. |

A revisão 01 do relatório técnico acompanha o firmware **1.4.0**: taxa de
amostragem do Modo Vibração de até 500 Hz, retransmissão de amostras perdidas
na transferência via BLE e os dois refinamentos do pipeline de análise
espectral (interpolação do pico em decibéis e piso de ruído medido em janela
local). A revisão 02 acompanha o firmware **1.6.0**: filtro adaptativo
"1-euro" na leitura contínua exibida na tela, e extremos (mín./máx.) medidos
pelo próprio firmware a 100 Hz, em caminho de filtragem separado do caminho
de exibição — ver `firmware/README.md`. A revisão 03 acompanha o firmware
**1.6.1**, após o primeiro ensaio em bancada: viés do giroscópio estimado por
eixo (o azimute travava em ±90° ao mudar a inclinação), orçamento de handles
do serviço BLE (as características do Modo Vibração e da versão não eram
registradas), retransmissão adaptativa, e a validação de ponta a ponta do
Modo Vibração via BLE (`sim/run.sh`).

## Procedimentos de teste e instalação (formulário padrão da empresa)

Caixa de identificação (título/tipo/revisão/folha/código/PN) e rodapé de
aprovação (feito por/aprovado por/projeto-contrato/página) em todas as
páginas, fonte Arial, sem capa/sumário separados — modelo fornecido pelo
usuário. Estrutura fixa em 5 seções: Objetivo, Equipamentos e materiais
utilizados, Setup, o procedimento propriamente dito, e Resolução de
problemas.

| Documento | Tipo | Rev. | Conteúdo |
|---|---|---|---|
| [procedimento-teste-funcional-bancada.docx](procedimento-teste-funcional-bancada.docx) | PTE | 00 | Teste funcional completo em bancada: instalação do software desktop (modo notebook), setup de teste e verificação de todos os modos de conexão e funcionalidades, com tabela de registro de resultados e resolução de problemas. |
| [procedimento-instalacao-software-python.docx](procedimento-instalacao-software-python.docx) | PTI | 00 | Instalação e verificação do software desktop (Python/PyQt5) em PC. |
| [procedimento-instalacao-software-android.docx](procedimento-instalacao-software-android.docx) | PTI | 00 | Compilação e instalação do aplicativo Android (Kotlin/Jetpack Compose). |
| [procedimento-gravacao-firmware.docx](procedimento-gravacao-firmware.docx) | PTG | 00 | Compilação e gravação (upload) do firmware do ESP32 via PlatformIO. |

Os códigos de tipo (PTE/PTI/PTG), revisão, folha, código e PN nas caixas de
identificação são inferência própria a partir do modelo mostrado (campos
deixados em branco no original) — ajustar conforme a numeração real da
empresa antes de uso formal.

**Instalação e gravação nunca usam internet no computador de destino**, em
nenhum dos três procedimentos (Python, Android, firmware) — a mídia sempre
traz tudo. O padrão de identificação da empresa é CD(1) DAD (Ambiente de
Desenvolvimento — só o IDE e as ferramentas usadas para criar/editar o
software, ex.: VSCodium/VS Code e um Python completo para desenvolvimento),
CD(2) DSF (Código Fonte) e CD(3) DSE (Executável). Para deixar isso à
prova de leitura apressada, os três procedimentos seguem a mesma estrutura:

- um **RESUMO RÁPIDO** (caixa de destaque) logo no início, com o fluxo do
  caso comum em 2-3 frases;
- os passos da instalação/gravação em si, sempre offline, com o caminho do
  operador final em primeiro lugar quando existe um caminho separado
  (Python: seção 4.1 = instalador Setup.exe; os caminhos de
  desenvolvimento/manutenção vêm depois, claramente rotulados como tais);
- a preparação da mídia (que exige internet) mora só no **ANEXO A**, ao
  final do documento, fora da sequência numerada — nunca é confundida com
  um passo da instalação.

O CD(2) DSF do software Python (usado na instalação para desenvolvimento
e manutenção, seção 4.3 do procedimento) inclui, pronta, uma distribuição
**Python 3.10 portátil (sem instalador)** — o runtime necessário para
rodar o software a partir do código-fonte — sem precisar ser baixado no
computador de destino, exceto quando a mídia for gerada de novo (ANEXO A).
**O CD(1) DAD não é usado em nenhum passo da instalação**, nem mesmo no
caminho de desenvolvimento/manutenção: contém só o(s) instalador(es) do
IDE (ex.: VSCodium/VS Code) e um instalador completo do Python — ambos
necessários para desenvolver/editar o software, mas não para apenas
instalá-lo e rodá-lo — sem código-fonte e sem os arquivos do CD(2) DSF.
Fica arquivado como garantia/regra da fábrica e é retirado do repositório
físico apenas em auditoria, nunca durante a instalação. O CD(3) DSE do
aplicativo Android deve incluir,
junto com o `.apk`, a pasta do **Android Platform Tools** (`adb.exe`) pelo
mesmo motivo — mídias antigas sem essa pasta ainda funcionam pela
instalação manual do `.apk` (sem `adb`), descrita como alternativa no
próprio procedimento.

**Para o operador final (sem conhecimento de programação), o único
caminho de instalação do software Python é o CD(3) DSE via instalador
(Setup.exe)** — cria ícone na Área de Trabalho e no Menu Iniciar, com a
logo da Avibras Aeroco, abrindo por duplo clique como qualquer programa
do Windows. Os caminhos por código-fonte (CD1+CD2) ou pasta executável
solta são só para desenvolvimento/manutenção do software — nenhum dos
dois cria ícone de atalho.

No firmware, o **CD(3) DSE** traz os três binários já compilados
(`firmware.bin`, `bootloader.bin`, `partitions.bin`) e o `esptool` — único
material necessário para a gravação em campo/fábrica (seção 4.2 do
procedimento). O **CD(2) DSF** (a pasta `firmware/` do projeto PlatformIO)
só é retirado do repositório físico quando for preciso recompilar ou
alterar o firmware, nunca durante a gravação. O **CD(1) DAD** (instalador
do VSCodium/VS Code com a extensão PlatformIO) segue a mesma regra dos
outros dois procedimentos — arquivado só por garantia/rastreabilidade,
nunca usado na gravação.

Na gravação de firmware, o `esptool` (standalone, no CD(3) DSE) precisa
estar acessível pelo **PATH do Windows** na máquina de gravação — o
procedimento traz o passo com `setx PATH` para isso (seção 3, passo 5);
sem isso, o comando `esptool` só funciona se o terminal for aberto
exatamente na pasta onde o executável foi copiado.

O procedimento de teste funcional (seção 3) e o de instalação do software
Python (seção 3) trazem cada um um diagrama em blocos do respectivo setup,
com fonte editável em vetor (SVG) em [`diagramas/`](diagramas/) — abre e
edita em Inkscape, Illustrator, ou colado no PowerPoint/Word como imagem
editável.

## Documentação técnica de referência

Mantida junto ao código, e atualizada primeiro (é a fonte da verdade de
engenharia — os entregáveis formais acima devem refletir o que está aqui, não
o contrário):

| Documento | Conteúdo |
|---|---|
| [pinout.md](pinout.md) | Mapeamento completo de pinos do ESP32 (MPU6050 via I²C, comunicação USB). |
| [fluxograma-python-app.md](fluxograma-python-app.md) | Arquitetura e fluxos de execução do software desktop (`python-app/`, PyQt5). |
| [`HARDWARE/README.md`](../HARDWARE/README.md) | Versão de consulta rápida da lista de materiais (BOM), com datasheets linkados. |
| README de cada módulo (`python-app/`, `android-app/`, `firmware/`) | Instalação, uso e detalhes de implementação específicos de cada software. |
