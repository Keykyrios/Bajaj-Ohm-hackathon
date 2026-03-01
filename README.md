# Bajaj OHM Stage 2 — Simulation Code

Simulation scripts for the **Vehicle-Level Control Concept** of an L5 electric auto-rickshaw with independent rear hub motors.

---

## Files

| File | Language | Description |
|------|----------|-------------|
| `Extensive_Matlab_Simulation.m` | MATLAB | Full vehicle dynamics simulation with yaw control and torque vectoring |
| `sim_step_steer.py` | Python | Transient tire dynamics — relaxation length lag simulation |
| `sim_qp_w.py` | Python | QP weighting parameter sigmoid transition visualization |

---

## `Extensive_Matlab_Simulation.m`

End-to-end MATLAB simulation of the 3-DOF delta-trike dynamics with active yaw control and differential torque allocation.

### What it does
- Simulates three operating scenarios:
  - **Case 1 — Normal Turning:** 6° step-steer input at 12 m/s for a loaded vehicle (760 kg).
  - **Case 2 — μ-Split Acceleration:** Full throttle on an asymmetric friction surface (left μ=0.2, right μ=0.8).
  - **Case 3 — Load Variation:** Compares yaw tracking between a light vehicle (450 kg) and a heavy vehicle (760 kg) under identical steering.
- Computes reference yaw rate from a kinematic bicycle model with understeer gradient, clamped by the friction limit.
- Applies a proportional corrective yaw moment (`ΔMz = -600 * e_γ`) and distributes it as differential torque across left/right rear hub motors.
- Clamps individual motor torques to ±40 Nm hardware limits and friction-circle bounds.
- Integrates vehicle position (X, Y) in the global frame and animates the trajectory in 2D.

### Vehicle Parameters
| Parameter | Value |
|-----------|-------|
| Yaw inertia `Iz` | 480 kg·m² |
| Wheelbase `L` | 2.0 m |
| Rear track width `Tw` | 1.2 m |
| Wheel radius `rw` | 0.22 m |
| Front/rear cornering stiffness | 30000 / 35000 N/rad |
| Max motor torque | 40 Nm |

### Outputs
- Yaw rate tracking plots (actual vs. reference)
- Left/right motor torque distribution plots
- Vehicle path (X vs. Y) plots
- 2D vehicle animation (normal turn case)

### How to run
```matlab
>> Extensive_Matlab_Simulation
```
Requires no toolboxes.

---

## `sim_step_steer.py`

Simulates the **transient tire force response** to a step-steer input, demonstrating the effect of tire relaxation length on lateral force buildup.

### What it does
- Applies a step slip angle (α = 0.15 rad) at t = 0.5 s.
- Computes the **steady-state** lateral force using the Pacejka Magic Formula:
  ```
  F̄_y = D · sin(C · arctan(B·α − E·(B·α − arctan(B·α))))
  ```
  with coefficients `B=7.0, C=1.4, D=5000, E=-0.2`.
- Computes the **transient** lateral force by applying a first-order lag governed by relaxation length:
  ```
  τ = L_relax / v_x = 0.45 / 20 = 0.0225 s
  τ · dF_y/dt + F_y = F̄_y
  ```
- Plots both curves to show the physical delay in tire force generation.

### Output
Saves `output_step_steer.png` — used as a figure in the LaTeX report (Section 3.2).

### How to run
```bash
python sim_step_steer.py
```
Requires `numpy`, `matplotlib`.

---

## `sim_qp_w.py`

Visualizes the **sigmoid transition logic** for the QP objective weighting parameter `W`.

### What it does
- Sweeps absolute yaw error `|e_γ|` from 0 to 1.5 rad/s.
- Computes the smooth sigmoid weight:
  ```
  W(|e_γ|) = 1 − 1 / (1 + exp(−k · (|e_γ| − e₀)))
  ```
  with threshold `e₀ = 0.2 rad/s` and steepness `k = 15`.
- When `|e_γ| ≈ 0` (straight-line cruise): `W → 1` (efficiency priority).
- When `|e_γ| > e₀` (stability intervention): `W → 0` (stability priority).
- Plots `W` vs. `|e_γ|` with the threshold line annotated.

### Output
Saves `output_qp_w.png` — used as a figure in the LaTeX report (Section 7.1).

### How to run
```bash
python sim_qp_w.py
```
Requires `numpy`, `matplotlib`.

---

## Dependencies

| Language | Packages |
|----------|----------|
| Python 3.x | `numpy`, `matplotlib` |
| MATLAB R2020a+ | No external toolboxes |
