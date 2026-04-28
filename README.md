# Quadrotor Simulation and Control Diagram

This repository contains a simple quadrotor model with altitude control and a companion block-diagram script.

## Project Files

- `quadrotorSim.py`: nonlinear quadrotor simulation with a PD altitude controller.
- `diagram.py`: block diagram generator for the control loop.
- `diagrama_cuadrirotor.png`: image output created by `diagram.py`.

## Requirements

- Python 3.10+
- Dependencies listed in `requirements.txt`

Install dependencies:

```bash
pip install -r requirements.txt
```

## How to Run

Run the simulation:

```bash
python quadrotorSim.py
```

Run the diagram script:

```bash
python diagram.py
```

When `diagram.py` runs, it saves the block diagram automatically as `diagrama_cuadrirotor.png` in the same folder.

## quadrotorSim.py: Detailed Description

### 1) State Vector

The simulation uses a 16-state vector:

- Position in inertial frame: `x, y, z`
- Body-frame linear velocity: `u_b, v_b, w_b`
- Euler angles (attitude): `phi` (roll), `theta` (pitch), `psi` (yaw)
- Body angular rates: `p, q, r`
- Rotor angular speeds: `w_m1, w_m2, w_m3, w_m4`

### 2) Kinematics

Two kinematic mappings are used:

1. Position kinematics:

	- A rotation matrix `R(phi, theta, psi)` is built.
	- Inertial position derivative is computed as `p_dot = R^T v`, where `v = [u_b, v_b, w_b]`.

2. Attitude kinematics:

	- The matrix `J_inv(phi, theta)` maps body rates `[p, q, r]` to Euler-angle rates.
	- `Theta_dot = J_inv * [p, q, r]`.

This is what converts velocities and angular rates into changes in position and orientation over time.

### 3) Dynamics

The model includes translational, rotational, and motor dynamics:

1. Translational dynamics:

	- Gravity in body frame is computed from attitude.
	- Total thrust from all rotors acts along body z-axis.
	- Net force `F = F_g + F_m`.
	- Acceleration in body frame is
	  `v_dot = F/m - omega x v`.

2. Rotational dynamics:

	- Moments from rotor thrust imbalance (`M_m`) generate roll/pitch torques.
	- Rotor drag torques (`Q`) generate yaw torque.
	- Gyroscopic rotor-body coupling (`M_gy`) is included.
	- Rotor acceleration reaction torque (`M_r`) is included.
	- Angular acceleration is
	  `omega_dot = I^{-1}(M - omega x (I*omega))`.

3. Motor dynamics (first-order):

	- Each rotor speed follows
	  `w_m_dot = (k_m*u - w_m)/t_m`.
	- This models finite motor response instead of instant rotor speed changes.

### 4) Control Law (Altitude PD)

The script controls altitude only (z-axis), using:

- `error_z = z_ref - z`
- `error_z_dot = -z_dot`
- `T_total_req = m*g - (Kp_z*error_z + Kd_z*error_z_dot)`

Then:

- Total thrust is saturated with `np.clip`.
- The thrust is split equally among the 4 rotors.
- Desired rotor speed is computed from `T = K_T*w^2`.
- The same motor command is applied to all rotors.

Because all rotors receive equal command, the controller primarily regulates vertical motion and does not actively track roll/pitch/yaw references.

### 5) Numerical Integration

The ODE is integrated with SciPy:

- Solver: `solve_ivp`
- Method: `RK45`
- Time span: `0 to 10 s`
- Samples: `1000`

Initial condition is all zeros (`X0 = 0`), so the vehicle starts at rest with zero attitude and zero rotor speeds.

### 6) What Happens When You Run It

Running `python quadrotorSim.py` does the following:

1. Simulates the full nonlinear model for 10 seconds.
2. Computes rotor thrust history from rotor speeds.
3. Displays a first figure with two subplots:
	- Linear position (`x, y`) and altitude (`-z`) plus altitude reference line.
	- Attitude angles (`roll, pitch, yaw`).
4. Displays a second figure with rotor thrusts (`T1, T2, T3, T4`).

The script saves a consolidated figure named `figura_completa.png` in the working directory and also opens interactive Matplotlib windows.

## Notes and Limitations

- This is a compact educational model focused on altitude control behavior.
- Euler-angle kinematics can become singular near `theta = +/- 90 deg`.
- The script currently uses fixed controller gains and no actuator saturation by voltage limits (only thrust clipping).
- Plot labels are currently mixed-language due to the original script naming and titles.

## Dependencies

- numpy
- scipy
- matplotlib
