"""Vista explodida (isometrica simplificada, com detalhes reais de conectores,
headers e terminais) do PAN-TILT METER na caixa Patola, com callouts numerados
1-7 ligados ao BOM (HARDWARE/README.md). Mesmas coordenadas/dimensoes do
modelo 3D SketchUp (projeto-mecanico/pan-tilt-meter-vista-explodida.skp).
Gera PNG e SVG a partir da mesma figura matplotlib, no mesmo estilo dos
diagramas existentes em docs/diagramas/."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

def iso(x, y, z):
    az = np.radians(45)
    el = np.radians(30)
    sx = (x * np.cos(az) + y * np.sin(az))
    sy = (-x * np.sin(az) + y * np.cos(az)) * np.sin(el) + z * np.cos(el)
    return sx, sy

def box_iso(cx, cy, cz, w, d, h):
    x0, x1 = cx, cx + w
    y0, y1 = cy, cy + d
    z0, z1 = cz, cz + h
    top = [(x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)]
    front = [(x0,y0,z0),(x1,y0,z0),(x1,y0,z1),(x0,y0,z1)]
    side = [(x1,y0,z0),(x1,y1,z0),(x1,y1,z1),(x1,y0,z1)]
    return top, front, side

def shade(hexcolor, factor):
    r = int(hexcolor[1:3], 16) * factor
    g = int(hexcolor[3:5], 16) * factor
    b = int(hexcolor[5:7], 16) * factor
    return "#%02x%02x%02x" % (min(255,int(r)), min(255,int(g)), min(255,int(b)))

def draw_box(ax, cx, cy, cz, w, d, h, color, z_bonus=0.0):
    top, front, side = box_iso(cx, cy, cz, w, d, h)
    for face, factor in ((top, 1.15), (front, 0.95), (side, 0.72)):
        pts = [iso(*p) for p in face]
        ax.add_patch(Polygon(pts, closed=True, facecolor=shade(color, factor),
                              edgecolor="#333333", linewidth=0.7, zorder=cz + z_bonus))

def draw_pin_row(ax, x0, y0, z0, count, pitch, color="#bcbcc2"):
    for i in range(count):
        draw_box(ax, x0 + i*pitch, y0, z0, 0.03, 0.03, 0.12, color, z_bonus=0.01)

# Dimensoes e posicoes (polegadas) - EXATAMENTE as do modelo 3D SketchUp
CASE_W, CASE_D = 6.3, 4.3
TRAY_H, LID_H = 1.8, 0.6
z_levels = [0, 2.05, 3.95, 5.95, 7.75, 9.35, 10.95, 12.75]

COLOR_CASE = "#363a3e"
COLOR_BATTERY = "#1c1e20"
COLOR_BATTERY_WRAP = "#c8a528"
COLOR_BATTERY_TERM = "#cdcdd2"
COLOR_PCB_GREEN = "#196c44"
COLOR_PCB_BLUE = "#144b94"
COLOR_CHIP = "#121214"
COLOR_METAL = "#bcbec3"
COLOR_BUTTON = "#0c0c0d"
COLOR_REG = "#161618"
COLOR_RESISTOR = "#beaf6e"
COLOR_LEAD = "#b4b4b9"
COLOR_CABLE = "#16161a"
COLOR_SCREW = "#a0a0a5"

fig, ax = plt.subplots(figsize=(10, 9.6))
ax.set_aspect("equal")
ax.axis("off")

parts = []       # peças numeradas (para callouts)
extra = []        # detalhes sem numero proprio

# 1 - Caixa base
parts.append((0, 0, 0, CASE_W, CASE_D, TRAY_H, COLOR_CASE, 1))
for (sx, sy) in [(2.06,2.06),(3.96,2.06),(2.06,2.86),(3.96,2.86),
                 (3.66,0.46),(4.96,0.46),(3.66,1.36),(4.96,1.36)]:
    extra.append((sx, sy, 2.05, 0.2, 0.2, 0.35, COLOR_SCREW))

# 2 - Bateria 9V (PP3) com terminais reais
bx, by, bz = 0.4, 0.4, z_levels[2]
parts.append((bx, by, bz, 1.9, 1.05, 0.65, COLOR_BATTERY, 2))
extra.append((bx+0.08, by+0.08, bz+0.228, 1.75, 0.63, 0.015, COLOR_BATTERY_WRAP))
extra.append((bx+0.133, by+0.095, bz+0.65, 1.634, 0.861, 0.07, COLOR_CHIP))
extra.append((bx+0.418, by+0.44, bz+0.72, 0.17, 0.17, 0.07, COLOR_BATTERY_TERM))
extra.append((bx+1.292, by+0.405, bz+0.72, 0.24, 0.24, 0.05, COLOR_BATTERY_TERM))

# 3 - Regulador PTN78020W sobre placa perfurada
pbx, pby, pbz = 3.6, 0.4, z_levels[3]
parts.append((pbx, pby, pbz, 1.6, 1.2, 0.07, COLOR_PCB_GREEN, 3))
reg_w, reg_d, reg_h = 1.25, 0.885, 0.35
rx, ry, rz = pbx+(1.6-reg_w)/2, pby+(1.2-reg_d)/2, pbz+0.07
extra.append((rx, ry, rz, reg_w, reg_d, reg_h, COLOR_REG))
extra.append((rx+reg_w+0.06, ry+reg_d/2-0.045, rz, 0.28, 0.09, 0.09, COLOR_RESISTOR))
extra.append((rx+reg_w-0.02, ry+reg_d/2-0.01, rz+0.04, 0.08, 0.02, 0.02, COLOR_LEAD))
extra.append((rx+reg_w+0.34, ry+reg_d/2-0.01, rz+0.04, 0.08, 0.02, 0.02, COLOR_LEAD))

# 4 - ESP32-DevKitC V4 (detalhado)
ex, ey, ez = 2.0, 2.0, z_levels[4]
esp_w, esp_d, esp_h = 2.2, 1.1, 0.062
parts.append((ex, ey, ez, esp_w, esp_d, esp_h, COLOR_PCB_GREEN, 4))
can_w, can_d, can_h = 0.62, 0.6, 0.18
extra.append((ex+esp_w-can_w-0.35, ey+(esp_d-can_d)/2, ez+esp_h, can_w, can_d, can_h, COLOR_METAL))
ant_w, ant_d = 0.3, 0.46
extra.append((ex+esp_w-ant_w-0.03, ey+(esp_d-ant_d)/2, ez+esp_h, ant_w, ant_d, 0.015, COLOR_PCB_GREEN))
extra.append((ex-0.09, ey+(esp_d-0.22)/2, ez+esp_h, 0.28, 0.22, 0.09, COLOR_METAL))
extra.append((ex+0.28, ey+0.08, ez+esp_h, 0.08, 0.08, 0.05, COLOR_BUTTON))
extra.append((ex+0.28, ey+esp_d-0.16, ez+esp_h, 0.08, 0.08, 0.05, COLOR_BUTTON))
draw_pin_row(ax, ex+0.2, ey+0.065, ez-0.11, 10, 0.2)
draw_pin_row(ax, ex+0.2, ey+esp_d-0.035, ez-0.11, 10, 0.2)

# 5 - MPU6050 / GY-521 (detalhado)
gx, gy, gz = 2.2, 3.3, z_levels[5]
gy_w, gy_d = 0.85, 0.65
parts.append((gx, gy, gz, gy_w, gy_d, 0.05, COLOR_PCB_BLUE, 5))
extra.append((gx+gy_w/2-0.08, gy+gy_d/2-0.12, gz+0.05, 0.16, 0.16, 0.08, COLOR_CHIP))
draw_pin_row(ax, gx+0.08, gy+gy_d-0.04, gz-0.11, 5, 0.14, color="#b8b8bd")

# 6 - Cabo USB
cx0, cy0, cz0 = 3.8, -0.5, z_levels[6]
parts.append((cx0, cy0, cz0, 0.47, 0.16, 0.2, COLOR_METAL, 6))
extra.append((cx0+0.035, cy0+0.13, cz0+0.03, 0.4, 0.1, 0.14, COLOR_CHIP))
extra.append((cx0-0.06, cy0-0.85, cz0-0.03, 0.26, 0.9, 0.26, COLOR_CABLE))

# 7 - Tampa
parts.append((-0.08, -0.08, z_levels[7], CASE_W+0.16, CASE_D+0.16, LID_H, COLOR_CASE, 7))

# --- Desenha extras primeiro (abaixo), depois pecas numeradas por cima ---
for cx, cy, cz, w, d, h, color in extra:
    draw_box(ax, cx, cy, cz, w, d, h, color)
for cx, cy, cz, w, d, h, color, num in parts:
    draw_box(ax, cx, cy, cz, w, d, h, color, z_bonus=0.02)

LEGEND = {
    1: "Caixa base (plastica, tipo Patola)",
    2: "Bateria 9V (PP3) + bloco de terminais",
    3: "Regulador PTN78020W + Rset 21kΩ",
    4: "ESP32-DevKitC V4 (WROOM-32, antena, USB, EN/BOOT)",
    5: "Sensor MPU6050 (GY-521) + header 8 pinos",
    6: "Cabo USB (plugue tipo A + trecho)",
    7: "Tampa da caixa (plastica)",
}

label_x = 12.3
label_y_start = 13.7
label_step = 1.6
for i, num in enumerate(sorted(LEGEND.keys(), reverse=True)):
    cx, cy, cz, w, d, h, color, n = next(p for p in parts if p[7] == num)
    top_center = (cx + w/2, cy + d/2, cz + h)
    gx, gy = iso(*top_center)
    ly = label_y_start - i * label_step
    ax.plot([gx, label_x - 0.3], [gy, ly], color="#555555", linewidth=0.8,
             linestyle=(0, (3, 2)), zorder=80)
    circ = plt.Circle((gx, gy), 0.19, facecolor="white", edgecolor="#333333",
                       linewidth=1.0, zorder=81)
    ax.add_patch(circ)
    ax.text(gx, gy, str(num), ha="center", va="center", fontsize=8,
            fontweight="bold", zorder=82)
    badge = plt.Circle((label_x - 0.3, ly), 0.21, facecolor="#2b6fb3",
                        edgecolor="none", zorder=81)
    ax.add_patch(badge)
    ax.text(label_x - 0.3, ly, str(num), ha="center", va="center", fontsize=9,
            color="white", fontweight="bold", zorder=82)
    ax.text(label_x + 0.15, ly, LEGEND[num], ha="left", va="center", fontsize=9.5)

ax.set_title("PAN-TILT METER — Vista explodida do conjunto mecanico",
             fontsize=13, fontweight="bold", pad=10)
ax.text(-2.0, -1.9, "Projecao isometrica — mesmas coordenadas do modelo 3D (SketchUp). Medidas internas em polegadas (caixa Patola 6,3\" x 4,3\").",
        fontsize=8, color="#555555")

ax.set_xlim(-2.2, 18.0)
ax.set_ylim(-1.4, 14.1)
fig.tight_layout()

fig.savefig("vista-explodida-pan-tilt-meter.svg", format="svg")
fig.savefig("vista-explodida-pan-tilt-meter.png", format="png", dpi=200)
print("OK")
