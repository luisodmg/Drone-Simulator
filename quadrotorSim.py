import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

m = 0.045
g = 9.81
l = 0.058

Ixx = 3.0738e-5
Iyy = 3.0849e-5
Izz = 5.9680e-5
Jr  = 5.897e-8

I = np.diag([Ixx, Iyy, Izz])
I_inv = np.linalg.inv(I)

K_T = 3.334e-8
K_Q = 1.058e-10
k_m = 803.9
t_m = 0.07

z_ref = -1.0

Kp_z = 0.3
Kd_z = 0.2

def quadrotor_dynamics(t, X):
    x, y, z = X[0:3]
    u_b, v_b, w_b = X[3:6]
    phi, theta, psi = X[6:9]
    p, q, r = X[9:12]
    w_m = X[12:16]

    R = np.array([
        [np.cos(theta)*np.cos(psi), np.cos(theta)*np.sin(psi), -np.sin(theta)],
        [np.sin(phi)*np.sin(theta)*np.cos(psi) - np.cos(phi)*np.sin(psi),
         np.sin(phi)*np.sin(theta)*np.sin(psi) + np.cos(phi)*np.cos(psi),
         np.sin(phi)*np.cos(theta)],
        [np.cos(phi)*np.sin(theta)*np.cos(psi) + np.sin(phi)*np.sin(psi),
         np.cos(phi)*np.sin(theta)*np.sin(psi) - np.sin(phi)*np.cos(psi),
         np.cos(phi)*np.cos(theta)]
    ])

    v = np.array([u_b, v_b, w_b])
    p_dot = R.T @ v

    error_z = z_ref - z
    error_z_dot = 0 - p_dot[2]

    T_total_req = m*g - (Kp_z*error_z + Kd_z*error_z_dot)
    T_total_req = np.clip(T_total_req, 0, 3*m*g)

    T_individual = T_total_req / 4
    w_des = np.sqrt(T_individual / K_T)
    u_volts = w_des / k_m
    u = np.array([u_volts, u_volts, u_volts, u_volts])

    wp_m = (k_m * u - w_m) / t_m

    T = K_T * (w_m**2)
    Q = K_Q * (w_m**2)

    F_m = np.array([0, 0, -(T[0]+T[1]+T[2]+T[3])])
    F_g = np.array([
        -m * g * np.sin(theta),
         m * g * np.cos(theta) * np.sin(phi),
         m * g * np.cos(theta) * np.cos(phi)
    ])
    F = F_g + F_m

    sqrt2_2 = np.sqrt(2) / 2
    M_m = np.array([
        sqrt2_2 * l * (T[1] + T[2] - T[0] - T[3]),
        sqrt2_2 * l * (T[0] + T[1] - T[2] - T[3]),
        Q[0] - Q[1] + Q[2] - Q[3]
    ])

    w_res = w_m[0] - w_m[1] + w_m[2] - w_m[3]
    M_gy = np.array([
        -Jr * q * w_res,
         Jr * p * w_res,
         0
    ])

    wp_res = wp_m[0] - wp_m[1] + wp_m[2] - wp_m[3]
    M_r = np.array([0, 0, -Jr * wp_res])
    M = M_m + M_gy + M_r

    w_vec = np.array([p, q, r])
    vp = (F / m) - np.cross(w_vec, v)
    wp = I_inv @ (M - np.cross(w_vec, I @ w_vec))

    J_inv = np.array([
        [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]
    ])
    Theta_dot = J_inv @ w_vec

    return np.concatenate((p_dot, vp, Theta_dot, wp, wp_m))

# Initial conditions and simulation
X0 = np.zeros(16)
t_span = (0, 10)
t_eval = np.linspace(t_span[0], t_span[1], 1000)

sol = solve_ivp(quadrotor_dynamics, t_span, X0, t_eval=t_eval, method='RK45')

# Extract results
t = sol.t
x, y, z = sol.y[0], sol.y[1], sol.y[2]
phi, theta, psi = sol.y[6], sol.y[7], sol.y[8]
w_m_res = sol.y[12:16]
T_res = K_T * (w_m_res**2)
altitud = -z

# ── Figure 1: All three plots in one figure ─────────────────────────────────
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12))

ax1.plot(t, x, label='x')
ax1.plot(t, y, label='y')
ax1.plot(t, altitud, label='Altitud (-z)', color='blue', linewidth=2)
ax1.axhline(-z_ref, color='r', linestyle='--', label='Referencia Z')
ax1.set_title('Posición Lineal (Control PD en Altitud)')
ax1.set_ylabel('m')
ax1.grid()
ax1.legend()

ax2.plot(t, phi, label='roll')
ax2.plot(t, theta, label='pitch')
ax2.plot(t, psi, label='yaw')
ax2.set_title('Orientación (Actitud)')
ax2.set_ylabel('rad')
ax2.grid()
ax2.legend()

ax3.plot(t, T_res[0], label='T1')
ax3.plot(t, T_res[1], '--', label='T2')
ax3.plot(t, T_res[2], ':', label='T3')
ax3.plot(t, T_res[3], '-.', label='T4')
ax3.set_title('Esfuerzo de Control (Empuje de Rotores)')
ax3.set_xlabel('Tiempo (s)')
ax3.set_ylabel('Empuje (N)')
ax3.grid()
ax3.legend()

plt.tight_layout()
plt.savefig('figura_completa.png', dpi=150)
plt.show()