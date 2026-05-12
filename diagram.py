import matplotlib.pyplot as plt
import matplotlib.patches as patches
from pathlib import Path

# Configuración inicial de la figura
fig, ax = plt.subplots(figsize=(12, 7))
ax.axis('off')

# Función para dibujar bloques rectangulares
def block(x, y, text, width=0.1, height=0.1):
    rect = patches.Rectangle((x - width/2, y - height/2), width, height, 
                             linewidth=1.5, edgecolor='black', facecolor='#d9eadd')
    ax.add_patch(rect)
    ax.text(x, y, text, ha='center', va='center', fontsize=12, fontweight='bold')

# Función para uniones sumadoras (círculos)
def sum_junction(x, y, sign1, sign2):
    circle = patches.Circle((x, y), 0.025, linewidth=1.5, edgecolor='black', facecolor='white')
    ax.add_patch(circle)
    # Símbolo central
    ax.text(x, y, r"$\Sigma$", ha='center', va='center', fontsize=10)
    # Signos
    ax.text(x - 0.035, y + 0.015, sign1, ha='center', va='center', fontsize=12) # Izquierda
    ax.text(x + 0.015, y - 0.035, sign2, ha='center', va='center', fontsize=12) # Abajo

# Función para flechas directas
def arrow(x_start, y_start, x_end, y_end, label="", label_pos=(0, 0.02)):
    ax.annotate("", xy=(x_end, y_end), xytext=(x_start, y_start),
                arrowprops=dict(arrowstyle="->", lw=1.5, color="black"))
    if label:
        x_mid = (x_start + x_end) / 2 + label_pos[0]
        y_mid = (y_start + y_end) / 2 + label_pos[1]
        ax.text(x_mid, y_mid, label, ha='center', va='bottom', fontsize=11, color='blue')

# ==========================================
# POSICIONAMIENTO DE ELEMENTOS (Nivel principal Y = 0.7)
# ==========================================
y_main = 0.7

# Nodos X (Izquierda a derecha)
x_ref = 0.05
x_kbar = 0.18
x_sum1 = 0.32
x_B = 0.46
x_sum2 = 0.60
x_int = 0.74
x_split = 0.82 # Punto de derivación
x_C = 0.90
x_out = 1.05

# Dibujar Bloques
block(x_kbar, y_main, r"$\overline{k}$")
block(x_B, y_main, "B")
block(x_int, y_main, r"$\int$")
block(x_C, y_main, "C")
block(x_sum2, y_main - 0.2, "A")   # Lazo A
block(x_B, y_main - 0.4, "K")      # Lazo K

# Dibujar Sumadores
sum_junction(x_sum1, y_main, "+", "-")
sum_junction(x_sum2, y_main, "+", "+")

# ==========================================
# DIBUJAR CONEXIONES Y FLECHAS
# ==========================================
# Camino Principal (Adelante)
arrow(x_ref, y_main, x_kbar - 0.05, y_main, r"$r$")
arrow(x_kbar + 0.05, y_main, x_sum1 - 0.025, y_main)
arrow(x_sum1 + 0.025, y_main, x_B - 0.05, y_main, r"$u$")
arrow(x_B + 0.05, y_main, x_sum2 - 0.025, y_main)
arrow(x_sum2 + 0.025, y_main, x_int - 0.05, y_main, r"$\dot{x}$")
arrow(x_int + 0.05, y_main, x_C - 0.05, y_main, r"$x$")
arrow(x_C + 0.05, y_main, x_out, y_main, r"$y$")

# Punto de derivación de x (Puntito negro)
ax.plot(x_split, y_main, 'ko', markersize=5)

# Lazo de Retroalimentación de la Dinámica Interna (Matriz A)
ax.plot([x_split, x_split], [y_main, y_main - 0.2], 'k-', lw=1.5)
ax.plot([x_split, x_sum2 + 0.05], [y_main - 0.2, y_main - 0.2], 'k-', lw=1.5)
ax.annotate("", xy=(x_sum2, y_main - 0.025), xytext=(x_sum2, y_main - 0.15),
            arrowprops=dict(arrowstyle="->", lw=1.5, color="black"))

# Lazo de Retroalimentación del Controlador (Matriz K)
ax.plot([x_split, x_split], [y_main - 0.2, y_main - 0.4], 'k-', lw=1.5)
ax.plot([x_split, x_B + 0.05], [y_main - 0.4, y_main - 0.4], 'k-', lw=1.5)
ax.plot([x_B - 0.05, x_sum1], [y_main - 0.4, y_main - 0.4], 'k-', lw=1.5)
ax.annotate("", xy=(x_sum1, y_main - 0.025), xytext=(x_sum1, y_main - 0.4),
            arrowprops=dict(arrowstyle="->", lw=1.5, color="black"))

# Título y guardado
plt.title("Diagrama de Bloques: Control por Retroalimentación de Estados\n", fontsize=14, fontweight='bold')
output_path = Path(__file__).with_name("diagrama_cuadrirotor_sf.png")
plt.savefig(output_path, format="png", dpi=300, bbox_inches="tight")

plt.show()