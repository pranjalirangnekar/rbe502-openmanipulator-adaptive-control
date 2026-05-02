# RBE 502 Final Project: Adaptive Control of OpenManipulator-X

This repository contains the software implementation for the RBE 502 Spring Final Project on adaptive control of the 4-DOF OpenManipulator-X robotic manipulator.

The project compares three joint-space controllers:

1. Computed Torque Control
2. Robust Dynamics-Based Control
3. Adaptive Control

Each controller is evaluated on two tasks:

1. Pose regulation
2. Trajectory tracking

At the current stage, the simulation side is complete and verified. Real-robot scripts/results are still pending and will be added after hardware testing.

---

## Team

- Pranjali Rangnekar, `prangnekar@wpi.edu`
- Jay Manish Patil, `jpatil1@wpi.edu`
- Harsh Shah, `hshah2@wpi.edu`

---

## Repository Base

This project is built on top of the OpenManipulator-X MATLAB repository provided for the class/project.

Original support files include:

- `Communication_Code/`
- `generated_dynamics/`
- `Identification/`
- `M_fun.m`
- `C_fun.m`
- `G_fun.m`
- `ViscousFriction_fun.m`
- `forward_dynamics.m`
- `example_joint_control.m`
- `example_current_control.m`
- `torque_to_current.m`
- `current_to_torque.m`

The provided dynamics model is:

```math
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau
```

where:

- `q` is the 4-DOF joint position vector
- `qdot` is the joint velocity vector
- `qddot` is the joint acceleration vector
- `tau` is the joint torque vector
- `M_fun.m`, `C_fun.m`, and `G_fun.m` evaluate the model terms

---

## Current Project Status

### Completed

- MATLAB repo setup
- Model verification using `M_fun`, `C_fun`, and `G_fun`
- Computed torque control simulation
- Robust control simulation
- Adaptive control simulation
- Pose regulation simulation for all controllers
- Trajectory tracking simulation for all controllers
- Summary table generation
- Final plot generation
- Output completeness checker

### Pending

- Real robot experiments
- Friction compensation decision for hardware
- Real robot summary plots/tables
- Final report update after hardware testing
- Demo videos

---

## Folder Structure

```text
OpenManipulator-X-main/
│
├── Communication_Code/
│   └── Original communication/hardware support code
│
├── generated_dynamics/
│   └── Generated dynamics functions
│
├── Identification/
│   └── Identified model parameters and related files
│
├── project_scripts/
│   ├── run_ct_regulation_sim.m
│   ├── run_ct_tracking_sim.m
│   ├── run_robust_regulation_sim.m
│   ├── run_robust_tracking_sim.m
│   ├── run_adaptive_regulation_sim.m
│   ├── run_adaptive_tracking_sim.m
│   ├── summarize_all_sim_results.m
│   ├── regenerate_all_sim_plots.m
│   └── check_simulation_outputs.m
│
├── project_utils/
│   ├── load_openmanipulator_params.m
│   └── build_numeric_regressor.m
│
├── project_results/
│   ├── sim_computed_torque_regulation/
│   ├── sim_computed_torque_tracking/
│   ├── sim_robust_regulation/
│   ├── sim_robust_tracking/
│   ├── sim_adaptive_regulation_safe/
│   ├── sim_adaptive_tracking_safe/
│   ├── summary/
│   └── final_plots/
│
├── archive_old/
│   └── Old or failed runs. Do not use these for final results.
│
└── README.md
```

---

## Important Result Folders

Use these folders for final simulation results:

```text
project_results/sim_computed_torque_regulation
project_results/sim_computed_torque_tracking
project_results/sim_robust_regulation
project_results/sim_robust_tracking
project_results/sim_adaptive_regulation_safe
project_results/sim_adaptive_tracking_safe
project_results/summary
project_results/final_plots
```

Do **not** use:

```text
archive_old/sim_adaptive_regulation_BAD_NAN
```

That folder contains an unstable adaptive controller test that produced NaN results. It was archived only for reference.

---

## How to Run the Code

Open MATLAB and set the working directory to the root of this repository:

```matlab
cd('C:\robot_control\final_project\OpenManipulator-X-main')
```

Add all folders to the MATLAB path:

```matlab
addpath(genpath(pwd));
```

Run individual simulations using:

```matlab
run(fullfile('project_scripts', 'run_ct_regulation_sim.m'))
run(fullfile('project_scripts', 'run_ct_tracking_sim.m'))

run(fullfile('project_scripts', 'run_robust_regulation_sim.m'))
run(fullfile('project_scripts', 'run_robust_tracking_sim.m'))

run(fullfile('project_scripts', 'run_adaptive_regulation_sim.m'))
run(fullfile('project_scripts', 'run_adaptive_tracking_sim.m'))
```

Generate the simulation summary table:

```matlab
run(fullfile('project_scripts', 'summarize_all_sim_results.m'))
```

Regenerate all final plots:

```matlab
run(fullfile('project_scripts', 'regenerate_all_sim_plots.m'))
```

Check that all required simulation outputs exist:

```matlab
run(fullfile('project_scripts', 'check_simulation_outputs.m'))
```

---

## Output Verification

The output checker should report:

```text
6/6 result .mat files: OK

computed_torque_regulation : 4 png files
computed_torque_tracking   : 4 png files
robust_regulation          : 4 png files
robust_tracking            : 4 png files
adaptive_regulation        : 5 png files
adaptive_tracking          : 5 png files
comparison                 : 3 png files
```

The adaptive cases have 5 plots because they include the additional estimated parameter evolution plot.

---

## Controllers Implemented

### 1. Computed Torque Control

The computed torque controller uses the nominal model directly.

Tracking error:

```math
e = q_d - q
```

```math
\dot{e} = \dot{q}_d - \dot{q}
```

Auxiliary input:

```math
v = \ddot{q}_d + K_d\dot{e} + K_p e
```

Control law:

```math
\tau = M(q)v + C(q,\dot{q})\dot{q} + G(q)
```

Scripts:

```text
project_scripts/run_ct_regulation_sim.m
project_scripts/run_ct_tracking_sim.m
```

---

### 2. Robust Dynamics-Based Control

The robust controller uses a filtered/sliding error variable.

Filtered error:

```math
s = \dot{e} + \Lambda e
```

Reference acceleration:

```math
v = \ddot{q}_d + \Lambda \dot{e}
```

Control law:

```math
\tau =
M(q)v + C(q,\dot{q})\dot{q} + G(q)
+ K_s s + \rho \tanh\left(\frac{s}{\epsilon}\right)
```

The `tanh` term is used instead of a discontinuous `sign` function to reduce chattering.

Scripts:

```text
project_scripts/run_robust_regulation_sim.m
project_scripts/run_robust_tracking_sim.m
```

---

### 3. Adaptive Control

The adaptive controller is based on a Slotine-Li style filtered tracking error.

Tracking error:

```math
e = q_d - q
```

```math
\dot{e} = \dot{q}_d - \dot{q}
```

Reference velocity and acceleration:

```math
\dot{q}_r = \dot{q}_d + \Lambda e
```

```math
\ddot{q}_r = \ddot{q}_d + \Lambda \dot{e}
```

Filtered error:

```math
s = \dot{q}_r - \dot{q}
```

Adaptive control law:

```math
\tau = Y(q,\dot{q},\dot{q}_r,\ddot{q}_r)\hat{\theta} + K_d s
```

Adaptation law:

```math
\dot{\hat{\theta}} =
\Gamma \frac{Y^T s}{1 + \|Y\|_F^2}
-\sigma(\hat{\theta}-\theta_{nom})
```

Additional safety/stability features used in simulation:

- normalized adaptation
- leakage term
- parameter projection
- torque saturation
- instability stop condition

Scripts:

```text
project_scripts/run_adaptive_regulation_sim.m
project_scripts/run_adaptive_tracking_sim.m
```

---

## Numeric Regressor Construction

The project dynamics can be represented in linearly parameterized form:

```math
Y(q,\dot{q},\ddot{q})\pi = \tau
```

The repository provides `M_fun.m`, `C_fun.m`, and `G_fun.m`, but does not provide a direct `Y_fun.m`.

Therefore, the regressor was constructed numerically in:

```text
project_utils/build_numeric_regressor.m
```

Method:

1. Keep geometry and gravity fixed.
2. Activate one dynamic parameter at a time.
3. Evaluate:

```math
M(q)\ddot{q}_r + C(q,\dot{q})\dot{q}_r + G(q)
```

4. Use the result as one column of the regressor matrix `Y`.

The adaptive parameter vector contains 16 dynamic parameters:

- 4 mass parameters
- 12 inertia parameters

The following parameters are kept fixed:

- geometric parameters
- gravity

---

## Simulation Tasks

### Task 1: Pose Regulation

The desired joint configuration is:

```math
q_d =
\begin{bmatrix}
0.25 & -0.35 & 0.35 & 0.15
\end{bmatrix}^T
\text{ rad}
```

Initial condition:

```math
q(0) =
\begin{bmatrix}
0 & 0 & 0 & 0
\end{bmatrix}^T
```

```math
\dot{q}(0) =
\begin{bmatrix}
0 & 0 & 0 & 0
\end{bmatrix}^T
```

---

### Task 2: Trajectory Tracking

The computed torque and robust tracking simulations use a sinusoidal desired trajectory:

```math
q_d(t) = q_{offset} + A \sin(\omega t)
```

with analytical velocity and acceleration:

```math
\dot{q}_d(t) = A\omega \cos(\omega t)
```

```math
\ddot{q}_d(t) = -A\omega^2 \sin(\omega t)
```

The adaptive tracking simulation uses a smoother cosine-based trajectory:

```math
q_d(t) = q_{offset} + A(1 - \cos(\omega t))
```

This gives zero initial desired velocity, which improves numerical stability for adaptive tracking.

---

## Simulation Results Summary

The simulation summary is saved in:

```text
project_results/summary/simulation_summary.csv
project_results/summary/simulation_summary.mat
```

Current summary:

| Controller      | Task       | Final Error | Max Error | Mean Error |
| --------------- | ---------- | ----------: | --------: | ---------: |
| Computed Torque | Regulation |  2.1048e-08 |   0.57446 |   0.026432 |
| Computed Torque | Tracking   |  1.1985e-05 |  0.014715 | 0.00066625 |
| Robust          | Regulation |   0.0005927 |   0.57446 |   0.025227 |
| Robust          | Tracking   |  0.00032469 | 0.0069946 |  0.0027116 |
| Adaptive        | Regulation |    0.061166 |   0.57446 |    0.15553 |
| Adaptive        | Tracking   |    0.044826 |    0.2486 |    0.12636 |

Error metric:

```math
\|e(t)\|_2 = \|q_d(t)-q(t)\|_2
```

---

## Final Plots

Final plots are stored in:

```text
project_results/final_plots/
```

Subfolders:

```text
computed_torque_regulation/
computed_torque_tracking/
robust_regulation/
robust_tracking/
adaptive_regulation/
adaptive_tracking/
comparison/
```

Each controller/task folder contains:

- joint position plots
- joint velocity plots
- torque plots
- tracking error norm plots

Adaptive folders also contain:

- estimated parameter evolution plots

Recommended plots for the report:

```text
project_results/final_plots/comparison/simulation_mean_error_comparison.png
project_results/final_plots/robust_tracking/robust_tracking_positions.png
project_results/final_plots/adaptive_tracking/adaptive_tracking_error_norm.png
project_results/final_plots/adaptive_tracking/adaptive_tracking_parameters.png
```

---

## Interpretation of Simulation Results

In simulation, computed torque control performs best because the same nominal dynamics are used in both the controller and the simulated plant.

Robust control also performs well and achieves low tracking error due to the stabilizing filtered error feedback and smooth robust term.

Adaptive control is stable but has higher error. This is expected because the adaptive controller uses conservative adaptation gains, parameter projection, and torque saturation to prevent instability and parameter drift. The adaptive controller is still important because it provides online parameter adjustment, which may become more useful on the real robot where model mismatch, friction, torque limits, backlash, and sensor noise are present.

---

## Real Robot Work

Real robot experiments are pending.

Recommended testing order:

1. Position-mode sanity check
2. Current-mode zero torque test
3. Computed torque regulation
4. Robust regulation
5. Adaptive regulation
6. Computed torque tracking
7. Robust tracking
8. Adaptive tracking

Do not start with trajectory tracking on hardware. Regulation should be tested first.

If friction compensation is needed on the real robot, use the provided friction model and apply it consistently to all controllers:

```math
\tau_{cmd} = \tau_{controller} + \tau_f
```

Friction compensation should be used only for real-robot experiments, not for simulation results.

---

## Notes for Teammates

Before running anything, make sure MATLAB is in the repository root:

```matlab
cd('C:\robot_control\final_project\OpenManipulator-X-main')
```

Then run:

```matlab
addpath(genpath(pwd));
```

To verify all simulation outputs:

```matlab
run(fullfile('project_scripts', 'check_simulation_outputs.m'))
```

If all checks show `[OK]`, the simulation package is complete.

Do not use anything from:

```text
archive_old/
```

unless debugging old failed runs.

