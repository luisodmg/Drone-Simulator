import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.signal import place_poles

# ==========================================
# PARÁMETROS DEL SISTEMA
# ==========================================
m = 0.068
g = 9.81
Ixx = 0.0686e-3
Iyy = 0.092e-3
Izz = 0.1366e-3

# ==========================================
# MODELO EN ESPACIO DE ESTADOS
# Estado: [x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot]
# ==========================================
A = np.zeros((12, 12))
A[0, 1] = 1.0; A[2, 3] = 1.0; A[4, 5] = 1.0
A[6, 7] = 1.0; A[8, 9] = 1.0; A[10, 11] = 1.0
A[1, 8] = g    # x_ddot = g * theta
A[3, 6] = -g   # y_ddot = -g * phi

B = np.zeros((12, 4))
B[5, 0] = 1/m
B[7, 1] = 1/Ixx
B[9, 2] = 1/Iyy
B[11, 3] = 1/Izz

C = np.zeros((4, 12))
C[0, 0] = 1.0  # x
C[1, 2] = 1.0  # y
C[2, 4] = 1.0  # z
C[3, 10] = 1.0 # psi

# ==========================================
# DISEÑO DEL CONTROLADOR
# ==========================================
# Polos
P_des = np.array([-15, -15, -12, -12, -20, -20, -10, -10, -18, -18, -8, -8])

# Calcular matriz de ganancias K usando asignación de polos
resultado_k = place_poles(A, B, P_des)
K = resultado_k.gain_matrix

# Calcular ganancia de pre-compensación k_bar (Diapositiva 19)
A_cl = A - B @ K
DC_gain = - C @ np.linalg.inv(A_cl) @ B 
k_bar = np.linalg.inv(DC_gain)

# ==========================================
# SIMULACIÓN CON NUEVA TRAYECTORIA
# ==========================================
def quadrotor_linear_dynamics(t, x_state):
    # Generación de Trayectoria (Figura de 8 Inclinada)
    # x oscila normal, y oscila al doble de velocidad (forma el 8)
    x_r = 2.0 * np.sin(0.2 * t)
    y_r = 2.0 * np.sin(0.4 * t) 
    
    # z tiene una altura base de 3 metros, y oscila +- 1.5 metros sincronizado con x
    z_r = 3.0 + 1.5 * np.sin(0.2 * t) 
    psi_r = 0.0
    
    # Vector de referencia
    r = np.array([x_r, y_r, z_r, psi_r])
    
    # Ley de control: u = -Kx + k_bar * r
    u = -K @ x_state + k_bar @ r
    
    # Dinámica lineal de lazo cerrado (Simulink)
    x_dot = A @ x_state + B @ u
    return x_dot

# Condiciones iniciales (todo en 0)
x0 = np.zeros(12)

# Tiempo de simulación: 70 segundos
t_span = (0, 70)
t_eval = np.linspace(t_span[0], t_span[1], 3000)

sol = solve_ivp(quadrotor_linear_dynamics, t_span, x0, t_eval=t_eval)

# Extraer resultados
t = sol.t
x_act, y_act, z_act = sol.y[0], sol.y[2], sol.y[4]
phi, theta, psi = sol.y[6], sol.y[8], sol.y[10]

# Referencias para graficar (Deben coincidir con las de la dinámica)
x_ref = 2.0 * np.sin(0.2 * t)
y_ref = 2.0 * np.sin(0.4 * t)
z_ref = 3.0 + 1.5 * np.sin(0.2 * t)

# ==========================================
# GRÁFICAS
# ==========================================
fig = plt.figure(figsize=(12, 10))

# 1. Gráfica 3D 
ax1 = fig.add_subplot(2, 2, 1, projection='3d')
ax1.plot(x_ref, y_ref, z_ref, 'k--', label='Deseada')
ax1.plot(x_act, y_act, z_act, 'b-', label='Actual')
ax1.set_xlabel('x - axis (m)')
ax1.set_ylabel('y - axis (m)')
ax1.set_zlabel('z - axis (m)')
ax1.set_title('Trayectoria 3D (Figura 8 Inclinada)')
ax1.legend()

# 2. Poses Lineales X, Y, Z
ax2 = fig.add_subplot(2, 2, 2)
ax2.plot(t, x_ref, 'k--', alpha=0.5)
ax2.plot(t, x_act, label='x', color='b')
ax2.plot(t, y_ref, 'k--', alpha=0.5)
ax2.plot(t, y_act, label='y', color='g')
ax2.plot(t, z_ref, 'k--', alpha=0.5)
ax2.plot(t, z_act, label='z', color='r')
ax2.set_xlabel('Tiempo (s)')
ax2.set_ylabel('Posición (m)')
ax2.set_title('Seguimiento de Posición')
ax2.legend()
ax2.grid()

# 3. Orientación (Actitud)
ax3 = fig.add_subplot(2, 2, 3)
ax3.plot(t, phi, label=r'Roll ($\phi$)')
ax3.plot(t, theta, label=r'Pitch ($\theta$)')
ax3.plot(t, psi, label=r'Yaw ($\psi$)')
ax3.set_xlabel('Tiempo (s)')
ax3.set_ylabel('Ángulos (rad)')
ax3.set_title('Orientación')
ax3.legend()
ax3.grid()

# 4. Señal de Control U (Fuerza y Torques)
U_hist = np.zeros((4, len(t)))
for i in range(len(t)):
    r_i = np.array([x_ref[i], y_ref[i], z_ref[i], 0.0])
    U_hist[:, i] = -K @ sol.y[:, i] + k_bar @ r_i

ax4 = fig.add_subplot(2, 2, 4)
ax4.plot(t, U_hist[0, :], label='Empuje $u_1$ (N)')
ax4.set_xlabel('Tiempo (s)')
ax4.set_ylabel('Empuje (N)')
ax4.set_title('Señal de Control $u_1$')
ax4.legend()
ax4.grid()

plt.tight_layout()
plt.savefig('reporte_sf_fig8.png', dpi=150)
plt.show()