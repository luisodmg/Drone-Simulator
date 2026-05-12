# Drone Assignments

This repository contains a small quadrotor control assignment built in Python.
It includes:

- a linear state-space simulation of a quadrotor with state-feedback control
- a block diagram generator for the feedback control structure
- the generated figures used in the report

## Project Files

- `quadrotorSim.py` - runs the quadrotor simulation, computes the control law, and saves `reporte_sf_final.png`
- `diagram.py` - draws the block diagram and saves `diagrama_cuadrirotor_sf.png`
- `requirements.txt` - Python dependencies needed to run the scripts

## What the Simulation Does

The simulation uses a linearized quadrotor model in state-space form with 12 states:

- position: `x, y, z`
- velocity: `x_dot, y_dot, z_dot`
- attitude: `phi, theta, psi`
- angular rates: `phi_dot, theta_dot, psi_dot`

The controller is built with pole placement using `scipy.signal.place_poles`. A precompensation gain is also computed so the system can track the reference trajectory.

The reference path is a spiral:

- `x = 2 cos(0.2 t)`
- `y = 2 sin(0.2 t)`
- `z = 0.2 t`
- `psi = 0`

## Requirements

Install the dependencies listed in `requirements.txt`:

```bash
pip install -r requirements.txt
```

## How to Run

Run the simulation:

```bash
python quadrotorSim.py
```

Run the diagram generator:

```bash
python diagram.py
```

If `python` does not work on your machine, try `python3` instead.

## Output Files

After running the scripts, you should get these images in the project folder:

- `reporte_sf_final.png`
- `diagrama_cuadrirotor_sf.png`

## Notes

- The simulation is based on a linear model, so it is meant for control design and analysis rather than full nonlinear flight dynamics.
- The scripts are self-contained and can be run independently.
- If you change the model parameters or desired poles, rerun `quadrotorSim.py` to regenerate the report figure.
