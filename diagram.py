import matplotlib.pyplot as plt
from pathlib import Path

fig, ax = plt.subplots(figsize=(10,6))

# Hide plot axes for a clean block diagram
ax.axis('off')

# Helper function to draw one labeled block
def block(x, y, text):
    ax.text(x, y, text, ha='center', va='center',
            bbox=dict(boxstyle="round", fc="lightblue", ec="black"))

# Main control-flow blocks
block(0.1, 0.5, "Referencia\nz_ref")
block(0.3, 0.5, "Controlador\nPD")
block(0.5, 0.5, "Distribución\nT1,T2,T3,T4")
block(0.7, 0.5, "Dinámica\nCuadrirotor")
block(0.9, 0.5, "Estados\nx,y,z,φ,θ,ψ")

# Forward arrows between blocks
ax.annotate("", xy=(0.25,0.5), xytext=(0.15,0.5),
            arrowprops=dict(arrowstyle="->"))
ax.annotate("", xy=(0.45,0.5), xytext=(0.35,0.5),
            arrowprops=dict(arrowstyle="->"))
ax.annotate("", xy=(0.65,0.5), xytext=(0.55,0.5),
            arrowprops=dict(arrowstyle="->"))
ax.annotate("", xy=(0.85,0.5), xytext=(0.75,0.5),
            arrowprops=dict(arrowstyle="->"))

# Feedback path from output to controller
ax.annotate("", xy=(0.3,0.45), xytext=(0.9,0.45),
            arrowprops=dict(arrowstyle="->"))
ax.text(0.6, 0.4, "z", ha='center')

plt.title("Diagrama de Bloques del Sistema de Control del Cuadrirotor")

output_path = Path(__file__).with_name("diagrama_cuadrirotor.png")
# Save diagram as a PNG file in this folder
plt.savefig(output_path, format="png", dpi=300, bbox_inches="tight")

plt.show()