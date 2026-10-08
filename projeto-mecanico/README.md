# projeto-mecanico — Vista explodida do conjunto

Modelo mecânico de montagem do inclinômetro/azimutímetro PAN-TILT METER:
uma caixa plástica tipo "Patola" (6,3" x 4,3" x 1,8" na base + 0,6" na
tampa) abrigando a bateria 9V, o regulador PTN78020W, o ESP32-DevKitC V4
e o sensor MPU6050 (GY-521), com o cabo USB saindo pela lateral.

Modelo 3D CAD simplificado, mas geometricamente fiel às peças reais —
sem recorrer a fotos de fabricante (que este ambiente não consegue
baixar de domínios externos): headers de pino contados, conector USB,
antena do módulo WROOM-32, botões EN/BOOT, terminais da bateria 9V
(positivo em pino, negativo em anel) e pinos SIP do regulador foram
todos modelados nas posições e proporções reais, não como caixas lisas
genéricas.

## Arquivos

| Arquivo | Conteúdo | Uso recomendado |
|---|---|---|
| [`vista-explodida-3d.png`](vista-explodida-3d.png) | Render do modelo 3D (SketchUp), câmera em 3/4, sombras e materiais | Figura "realista" no relatório técnico — mostra a forma real das peças |
| [`vista-explodida-pan-tilt-meter.svg`](vista-explodida-pan-tilt-meter.svg) | Vista explodida vetorial, mesmas coordenadas do modelo 3D, com callouts numerados 1-7 | Figura editável (Inkscape/Illustrator/Word) para o relatório e o procedimento de teste — mesma convenção de [`docs/diagramas/`](../docs/diagramas/) |
| `vista-explodida-pan-tilt-meter.png` | Versão raster (200 DPI) do SVG acima | Inserção rápida onde SVG não é aceito |
| [`gerar_vista_explodida.py`](gerar_vista_explodida.py) | Script Python/matplotlib que gera o SVG e o PNG acima | Reexecutar após qualquer mudança de hardware/BOM (`python3 gerar_vista_explodida.py`) |

## Legenda (numeração ligada à BOM)

| # | Item | Referência na BOM |
|---|---|---|
| 1 | Caixa base (plástica, tipo Patola) | — (gabinete, fora da lista de materiais do projeto eletrônico, ver seção 3 de [`HARDWARE/README.md`](../HARDWARE/README.md)) |
| 2 | Bateria 9V (PP3) + bloco de terminais | Seção 2 de `HARDWARE/README.md` (fonte de alimentação, opção pilha 9V) |
| 3 | Regulador PTN78020W + resistor Rset 21 kΩ | Item 6 de `HARDWARE/README.md` |
| 4 | ESP32-DevKitC V4 (módulo WROOM-32, antena, USB, botões EN/BOOT) | Item 1 de `HARDWARE/README.md` |
| 5 | Sensor MPU6050 (módulo GY-521) + header 8 pinos | Item 3 de `HARDWARE/README.md` |
| 6 | Cabo USB (plugue tipo A + trecho) | Itens 4/5 de `HARDWARE/README.md` |
| 7 | Tampa da caixa (plástica) | — (gabinete) |

Os parafusos/pilares de fixação (8x) aparecem no desenho sem numeração
própria — são itens de fixação genéricos, não um componente eletrônico
da BOM.

## Sobre o modelo 3D fonte (.skp)

O modelo foi construído no SketchUp (sessão em nuvem). Este ambiente não
consegue baixar o arquivo `.skp` para o repositório — o proxy de rede
bloqueia o domínio de download do serviço. O arquivo fonte (editável,
com todas as peças, materiais e câmera configurados) continua disponível
através da sessão que o gerou; peça o link de download a qualquer
momento caso precise editá-lo diretamente no SketchUp.

## Manutenção

Se a caixa, a posição dos componentes ou a BOM mudarem, atualizar:

1. As coordenadas/dimensões no modelo 3D (nova sessão SketchUp);
2. As mesmas coordenadas em [`gerar_vista_explodida.py`](gerar_vista_explodida.py) (os comentários no código indicam onde cada peça é posicionada);
3. Esta tabela de legenda, se a numeração ou os itens da BOM mudarem.
