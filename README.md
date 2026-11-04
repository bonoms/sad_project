# AURA — 6U CubeSat attitude dynamics and sensor modelling

This README describes the project structure, model implementation and intended behaviour. Inspect the current source files and model when making changes.

## 1. Project context

University spacecraft attitude dynamics project in **MATLAB and Simulink**, inspired by the **GOMX-4B** 6U CubeSat. This is an educational adaptation, not a reconstruction of flight software, CAD, calibration or telemetry.

The scenario is an uncontrolled spacecraft starting at rest in a near-polar, approximately Sun-synchronous low Earth orbit. The model includes environmental disturbances and an experimental star-tracker measurement model. There is no implemented closed-loop attitude estimator/controller or magnetic-coil actuator in the inspected top-level model.

**Current priority:** understand and validate the star-tracker measurement model, especially rotation-error composition, axis conventions, sampling and invalid-measurement handling, before integrating an estimator.

## 2. Repository and running

```text
sad/
├── main.m                 # Current run script: parameters → simulation → animation
├── aura_mission.slx       # Main model, including sensors/star_tracker
└── library/
    ├── library.slx        # Custom dynamics, kinematics and environment blocks
    ├── astroConstants.m   # Physical constants lookup
    └── slblocks.m         # Optional Library Browser registration: SAD Project
```

Use **`main.m`** as the entry point. The custom library is `library/library.slx`, and links refer to `library/...`.

Requirements: MATLAB and Simulink. Numerical checks were executed with R2026b. A three-orbit simulation completed before the gravity-gradient correction; the corrected gravity-gradient implementation and current tracker noise were subsequently tested separately. These checks do not establish full physical validation of the mission model.

Open `main.m` and Run. It:
1. Executes `clear`, `close all`, `clc`.
2. Derives `projectRoot` from its own location, adds the root and `library/` to the path, and loads the bundled library.
3. Defines physical, spacecraft, sensor and initial-condition variables in the base workspace.
4. Simulates with fixed-step `ode4`, `fixedStep = 0.1 s`, `stopTime = 3*T`.
5. Reads `out.A_BN` and animates the body axes, displaying every 100th logged sample.

### Workspace pitfalls

- Opening an SLX does not populate the base workspace. The inspected model's `InitFcn` is empty and its model workspace has no variables.
- Pressing Run in Simulink therefore requires initialization in **the same MATLAB session** first. Run all parameter sections of `main.m` through `fixedStep`, stopping before the `sim` call if only initialization is wanted.
- Running just the first section deletes variables via `clear` without rebuilding later parameters.
- Names are case-sensitive: `fixedStep`, `e0`, `omega0`, `I_inv`, `j_B`, `A_mis_sT`.
- Separate MATLAB sessions have independent workspaces and loaded model copies, even when sharing disk files.
- Do not use `main.m` as `InitFcn`: it calls `sim`, which can recursively launch simulation. A future automatic initializer should be parameter-only and contain neither `sim` nor `clear`.

## 3. Physical model, frames and truth path

`N` denotes the inertial frame and `B` the spacecraft body frame. The true DCM **`A_BN` maps inertial column vectors to body components**: `v_B = A_BN*v_N`. Angular velocity is body relative to inertial, expressed in body axes, in rad/s. Angles and trigonometric inputs are in radians.

Actual top-level signal flow:

```text
Environment(A_BN) → disturbance torque → dynamics → omega
        ↑                                     │
        │                  Euler 312/313 Kinematics(omega,e0)
        │                                     ↓
        └──────────────────────── Euler 312 to DCM → A_BN
                                                    │
                                       sensors(A_BN,omega)
```

`A_BN` also feeds the attitude To Workspace block. Sensor errors do not feed back into the environment or true dynamics. Preserve this separation when adding estimation/control.

The active attitude block is the linked **`Euler 312/313 Kinematics`** from `library/Euler 312//313 Kinematics`, followed by a 312-to-DCM converter. The library contains switched Euler coordinates; this is not direct DCM-state integration. Euler 312 divides by `cos(theta312)`, while Euler 313 divides by `sin(theta313)`. A prior inactive-313 denominator guard is part of the baseline library; it does not remove Euler singularities generally.

A constant-rate numerical test of the saved kinematics, using spherical inertia and zero disturbances, matched the exact rotation before the first handover but accumulated attitude lag across repeated handovers. At 1°/s about the first coordinate axis over 1,000 s, maximum attitude error was **1.10° with a 0.1 s step** and **0.55° with a 0.05 s step**. This is consistent with lost propagation during handovers; the precise reset/enable scheduling cause has not been established. Orthogonality alone does not detect this error, and reducing the step does not resolve its underlying cause.

### Current main.m parameters

| Quantity | Variable / value |
|---|---|
| Inertia [kg·m²] | `I = diag([0.087 0.067 0.033])`; `I_inv = inv(I)` |
| Orbit size | `height = 449 km`; `sma = R_E + height` |
| Eccentricity | `ecc = 0.002` |
| Inclination | `inc = deg2rad(97.32)` |
| Initial true anomaly | `ta0 = 0` |
| RAAN / argument of perigee declarations | `raan = 0`, `aop = 0`; declarations alone do not prove model use |
| Period | `n = sqrt(mu_E/sma^3)`, `T = 2*pi/n` |
| Initial body rate | `omega0 = [omegax0;omegay0;omegaz0]*0`, therefore zero despite each component being declared as 1°/s |
| Initial attitude | `e0 = [0;0;0]`, interpreted as 312 angles; aligned body/inertial axes |
| Retained declaration | `A0 = eye(3,3)`; not the active top-level initialization input |
| Residual dipole [A·m²] | `j_B = [0.01 0.05 0.01]`; disturbance dipole, not commanded coil current |

Physical constants are obtained from `astroConstants`: `mu_E=astroConstants(13)`, `R_E=astroConstants(23)`, `f_E=astroConstants(43)`, `omega_E=2*pi/(astroConstants(53)*3600)`, `eps_E=deg2rad(astroConstants(63))`. Earlier direct lookup gave `mu_E=398600.433 km³/s²`, `R_E=6371.01 km`, `omega_E≈7.29210659088e-5 rad/s`. `Re_E=3*R_E/(3-f_E)` and `dipole_tilt_E=deg2rad(11.5)` are also declared.

Orbital distances use **km**; geometry, inertia and torques use **SI**, with torque in **N·m**. A reference mass of approximately 8 kg is mission context, not a current rotational-model input. All these numerical assumptions require separate physical validation before attribution to GOMX-4B.

### Environmental disturbances

- Keplerian orbital propagation with nonzero eccentricity.
- Gravity-gradient torque: `M_GG = (3*mu_E/norm(r)^3) * cross(rhat_B, I*rhat_B)`, where `rhat_B` is the outward radial unit vector expressed in body coordinates. The current saved wiring uses `mu_E` in the numerator and the cubed radius in the denominator. A nonaligned initial-attitude test with other disturbances disabled agreed with an independent torque calculation to **8.9e-7 relative error**, using a 0.001 s finite-difference acceleration.
- Residual magnetic-dipole torque using an Earth-dipole field model.
- Solar radiation pressure (SRP), evaluated separately on six faces and summed.

Face order: **+X, −X, +Y, −Y, +Z, −Z**. `N_B` stores outward unit normals as columns.

```matlab
Area  = [0.02 0.02 0 0.03 0.06 0.06]; % m²; +Y intentionally has zero area in current data
rho_s = [0.15 0.5 0.5 0.5 0.5 0.5];
rho_d = [0.05 0.05 0.05 0.05 0.05 0.05];
r_F = [0.05 -0.25 0     0     0    0;
       0     0    0.10 -0.10  0    0;
       0     0    0     0     0.05 -0.05]; % m, relative to centre of mass
```

For each face, SRP computes the illuminated projected area using `max(dot(normal,sunDirectionBody),0)`, applies absorption/specular/diffuse force terms, then `cross(r_F,force)`. Irradiance is hardcoded as `1358 W/m²`, speed of light as `299792458 m/s`. No Earth eclipse or mutual spacecraft shadowing is included in that subsystem.

The linked Sun Direction block uses a uniform annual angle `lambda=2*pi*t/31558149.76` and direction `[cos(lambda); cos(0.4091)*sin(lambda); sin(0.4091)*sin(lambda)]` in inertial axes. It has no calendar-date phase input. `T_sun` and `eps_E` in `main.m` do not override those hardcoded library constants.

## 4. Star tracker: exact current implementation

Path: **`aura_mission/sensors/star_tracker`**.

Inputs:
- `Ab_n`: true 3×3 `A_BN`.
- `w`: true 3-component body angular velocity.

Outputs:
- `Ab_n_star_tracker`: perturbed/sample-selected 3×3 matrix.
- `valid_measure_bool`: numeric 0/1 validity indicator, implemented with double Constants. The subsystem annotation documents the intended validity behaviour and the instantaneous-rate approximation.

### Parameters in main.m

| Parameter | Current setting / interpretation |
|---|---|
| `fi_mis_sT`, `theta_mis_sT`, `psi_mis_sT` | Each `1e-7`; intended small-angle misalignment in radians, comment says 312 notation |
| `fi_acc` | `deg2rad(0.02/3)` ≈ `1.1635528e-4 rad` |
| `theta_acc` | Same as `fi_acc` |
| `psi_acc` | `deg2rad(0.06/3)` ≈ `3.4906585e-4 rad`; labelled roll accuracy, without a physical roll/boresight-axis definition |
| `max_slew_rate` | `deg2rad(0.3)` ≈ `0.00523598776 rad/s` |

The `/3` assumes the stated angular accuracies are **3-sigma** specifications. No sensor datasheet or physical sensor-frame/boresight convention is currently specified for these values; treat them as modelling inputs requiring verification.

The script defines exactly:

```matlab
A_mis_sT = [1           fi_mis_sT   -theta_mis_sT;
           -fi_mis_sT  1            psi_mis_sT;
            fi_mis_sT -psi_mis_sT   1];
```

Entry (3,1) uses `fi_mis_sT`, not `theta_mis_sT`; equal current values hide this distinction. This is a first-order small-angle matrix, not an exact finite rotation. `sT_bias = zeros(3,3)` is a separately declared additive matrix bias.

### Noise and matrix construction

Three Random Number blocks have mean `0`, variance `1`, and sample time **0.1 s**, with distinct seeds:

| Random Number block | Seed | Gain | Error subsystem input |
|---|---|---|---|
| `fi_noise` | `2` | `psi_acc` | `psi` |
| `theta_noise1` | `1` | `theta_acc` | `theta` |
| `psi_noise2` | `0` | `fi_acc` | `fi` |

The gains apply the declared standard deviations directly; there is no additional `/sqrt(2)` factor. Block names alone do not establish angular-axis meaning.

`skew_symmetrix_matrix` (existing spelling) constructs the following matrix from its **port variables**:

```text
E = [ 1   fi  -psi;
     -fi   1   theta;
      psi -theta  1 ]
```

The saved candidate measurement is **`A_mis_sT * A_BN * E + sT_bias`**: angular noise multiplies truth on the right, misalignment multiplies on the left, and the separate matrix bias is added afterwards. Right-side noise acts in the input/inertial coordinates of `A_BN`; left-side misalignment acts in its output coordinates. A physical sensor-frame interpretation requires an explicit mounting/boresight convention.

For the convention `E ≈ I - skew(delta)`, the port-to-axis mapping is **`delta = [theta; psi; fi]`**, with axes meaning the first, second and third matrix coordinates—not an independently defined physical roll/pitch/yaw frame. The larger `psi_acc` uncertainty therefore acts about the second coordinate axis. This is not an axis-assignment defect without a conflicting physical frame definition.

A fixed, identity-attitude test with zero disturbances evaluated **10,001 acquisition epochs**. Measured rotation-vector standard deviations about the first/second/third coordinates were **24.10 / 72.32 / 24.03 arcsec**, consistent with the configured amplitudes; maximum absolute sample cross-correlation was approximately **0.015**. This is a finite-sample statistical check, not proof of independence or datasheet accuracy. The first-order error construction is not exactly orthogonal; a prior mission run measured maximum tracker orthogonality error of **2.14e-6** in Frobenius norm.

The result is sampled by `1Hz_sampler`, a Zero-Order Hold with **sample time 1 s**. Noise generation is therefore at 10 Hz while the attitude sample is held at 1 Hz; these are distinct settings and are not both controlled by `fixedStep`.

### Slew-rate gating and validity

`Vector Norm` computes `norm(w)`. Both selection Switches test **`norm(w) > max_slew_rate`**; equality is treated as valid.

- At acceptable rate: output the current 1 Hz sampled matrix, validity `1`.
- Above the limit: output the stored selected measurement, validity `0`.
- The Unit Delay receives feedback from the selection Switch output, not the raw sampler. Its initial condition is `eye(3,3)` and its sample time is inherited (`-1`). Before the first accepted measurement, identity is only a placeholder.

This feedback preserves the last accepted value through sustained invalidity. In the earlier three-orbit run, the measurement stayed exactly unchanged over **130,111 adjacent sample pairs with both validity values zero**. Separate zero-rate, exact-threshold and just-above-threshold tests confirmed the comparison boundary.

As documented in the subsystem annotation, the rate check follows the upstream integration rate rather than modelling validity over a finite camera exposure. It is not explicitly sampled together with the 1 Hz candidate matrix. The flag therefore describes the instantaneous rate condition, not a fresh-acquisition event or a guarantee that the rate stayed acceptable throughout exposure. A future estimator must distinguish fresh accepted observations from held data.

### Sensor container

`sensors/gyro` is currently a placeholder with two inputs and an unconnected output. `sensors/sensors_out` is also unconnected internally, and the top-level sensors output is not consumed. The tracker measurement is currently accessed through logging, not an estimator. The tracker validity output is logged internally but is not routed to a combined external sensor bus.

## 5. Logged outputs and how to inspect them

| SimulationOutput field | Source / configured format |
|---|---|
| `out.tout` | Solver time vector |
| `out.A_BN` | True attitude, Array; expected 3×3×N for time-varying matrix history |
| `out.omega` | True body angular velocity, Array |
| `out.Ab_n_star_tracker` | `sensors/To Workspace`, **Structure With Time** |
| `out.sT_valid_measure_bool` | Tracker validity To Workspace, **Timeseries** |

Inspect `who(out)`, `class`, `size`, timestamps and actual structure fields before accessing data. Sensor samples and solver samples need not have matching lengths. `main.m` currently animates truth only; it does not compute tracker error statistics or plot validity.

If the animation gets a single 3×3 matrix instead of a history, inspect upstream connectivity and logging rather than blindly changing indexing. Keep `A_BN` (truth) distinct from `Ab_n_star_tracker` (measurement).

## 6. Development guidelines

- Document block connections, equations, frames and units. Keep changes focused and avoid unrelated architectural additions.
- Inspect Git status and the actual saved model first. Do not assume previous findings or old block SIDs remain valid.
- Save pending model edits before replacing or reloading model files. Do not overwrite newer saved changes with a stale in-memory model.
- Use `main.m` as the entry point.
- Distinguish intended behaviour, inspected implementation, structural checks and executed numerical tests. A successful compile is not proof of correct sensor statistics or physics.
- Preserve user wiring and make small scoped edits. Back up SLX files before multi-step changes. Do not commit, push, merge or discard work without authorization.
- SLX files are tracked with **Git LFS** (`*.slx filter=lfs`). Install/initialize LFS before checkout/commit; otherwise apparent binary modifications or pointer-only files can mislead. `git lfs fsck` checks local object integrity.
- Inspect referenced library definitions when reviewing linked blocks. Resolve exact block paths or Simulink identifiers rather than guessing names containing `/`.

## 7. Mission references

- [ESA: GOMX-4B mission overview](https://www.esa.int/Enabling_Support/Space_Engineering_Technology/Technology_CubeSats/GOMX-4B).
- Holst et al., [*Attitude and Orbit Control Results of the GOMX-4 Tandem CubeSat Mission*](https://davidgerhardt.com/publications/GOMX4_IAC2018.pdf), IAC-18-B4.6B.11, 2018.

These sources motivate the mission concept; they do not validate the current geometry, inertia, sensor accuracy or calibration assumptions.
