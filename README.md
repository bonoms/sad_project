# AURA — 6U CubeSat attitude dynamics and sensor modelling

This README is a self-contained project handoff for a new contributor or AI assistant. It records the **actual saved implementation**, its intended behaviour, and known gaps separately. Read it before proposing changes; then inspect the current files because the model may have evolved.

## 1. Project context and active work

University spacecraft attitude dynamics project in **MATLAB and Simulink**, inspired by the **GOMX-4B** 6U CubeSat. This is an educational adaptation, not a reconstruction of flight software, CAD, calibration or telemetry.

The scenario is an uncontrolled spacecraft starting at rest in a near-polar, approximately Sun-synchronous low Earth orbit. The model includes environmental disturbances and an experimental star-tracker measurement model. There is no implemented closed-loop attitude estimator/controller or magnetic-coil actuator in the inspected top-level model.

**Branch being documented: `implement/star_sensor`.** At inspection its latest commit was `79aac22` (`Update aura_mission model`), following `c4ba890` (slew-rate limit and validity) and `55ad146` (star-tracker misalignment and accuracy). These are inspection anchors, not a requirement that HEAD stay there.

Other local branches:
- `main`: baseline spacecraft/environment implementation.
- `bug/division-by-zero-313`: separate experimental enabled-integrator/Euler-switching work. Do not assume its `Euler 312/313 Kinematics1` architecture is present here or merge it implicitly. That branch's initial-condition handover implementation was subsequently rolled back during development.

**Current priority:** understand and validate the star-tracker measurement model, especially rotation-error composition, axis conventions, sampling and invalid-measurement handling, before integrating an estimator.

## 2. Repository and running

```text
sad/
├── main.m                 # Current run script: parameters → simulation → animation
├── aura_mission.slx       # Main model, including sensors/star_tracker
├── library/
│   ├── library.slx        # Custom dynamics, kinematics and environment blocks
│   ├── astroConstants.m   # Physical constants lookup
│   └── slblocks.m         # Optional Library Browser registration: SAD Project
└── project.m              # Legacy tracked script on this branch; do not run
```

Use **`main.m`**, not `project.m`. The legacy file expects the old `project_lib.slx` and older variable/output names. Its presence on this branch does not make it the entry point. The library is now `library/library.slx`, and links refer to `library/...`.

Requirements: MATLAB and Simulink; earlier execution was tested with R2026b. The current branch's sensor behaviour is not certified by those older tests. This README update inspected source, wiring and block parameters; it did **not** run a new full mission simulation or statistical sensor-validation suite.

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
- AI/MCP MATLAB sessions and the user's desktop MATLAB have separate workspaces and loaded model copies, even when sharing disk files.
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
- Gravity-gradient torque.
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
- `valid_measure_bool`: numeric 0/1 validity indicator. Despite its name, the Switch uses double Constants; Boolean typing has not been established.

### Parameters in main.m

| Parameter | Current setting / interpretation |
|---|---|
| `fi_mis_sT`, `theta_mis_sT`, `psi_mis_sT` | Each `1e-7`; intended small-angle misalignment in radians, comment says 312 notation |
| `fi_acc` | `deg2rad(0.02/3)` ≈ `1.1635528e-4 rad` |
| `theta_acc` | Same as `fi_acc` |
| `psi_acc` | `deg2rad(0.06/3)` ≈ `3.4906585e-4 rad`; script assumes roll axis is Z |
| `max_slew_rate` | `deg2rad(0.3)` ≈ `0.00523598776 rad/s` |

The `/3` assumes the stated angular accuracies are **3-sigma** specifications. No sensor datasheet is currently cited for these values or the roll-axis assumption; treat them as modelling inputs requiring verification.

The script defines exactly:

```matlab
A_mis_sT = [1           fi_mis_sT   -theta_mis_sT;
           -fi_mis_sT  1            psi_mis_sT;
            fi_mis_sT -psi_mis_sT   1];
```

Do not silently rewrite this when documenting: entry (3,1) uses `fi_mis_sT`, not `theta_mis_sT`. With equal current values this distinction is hidden.

### Noise and matrix construction

Three Random Number blocks (`fi_noise`, `theta_noise1`, `psi_noise2`) have mean `0`, variance `1`, seed parameter `0`, and sample time **0.1 s**. Their Gain blocks multiply by `fi_acc/sqrt(2)`, `theta_acc/sqrt(2)`, `psi_acc/sqrt(2)` respectively. Thus each scalar stream's marginal standard deviation after scaling is the corresponding declared accuracy divided by `sqrt(2)`, not the declared accuracy itself. Distinct seeds/independence have not been verified.

Actual wiring crosses the first and third labelled noise channels:
- `psi_acc`-scaled noise → error subsystem input `fi`.
- `theta_acc`-scaled noise → input `theta`.
- `fi_acc`-scaled noise → input `psi`.

`skew_symmetrix_matrix` (existing spelling) constructs the following matrix from its **port variables**:

```text
E = [ 1   fi  -psi;
     -fi   1   theta;
      psi -theta  1 ]
```

The Matrix Multiply block computes **`A_BN * E`** (right multiplication). The next **Sum** block computes **`A_BN * E + A_mis_sT`**. This is the saved implementation, not an endorsed rotation-error model.

The result is sampled by `1Hz_sampler`, a Zero-Order Hold with **sample time 1 s**. Noise generation is therefore at 10 Hz while the attitude sample is held at 1 Hz; these are distinct settings and are not both controlled by `fixedStep`.

### Slew-rate gating and validity

`Vector Norm` computes `norm(w)`. Both selection Switches test **`norm(w) > max_slew_rate`**; equality is treated as valid.

- At acceptable rate: output the current 1 Hz sampled matrix, validity `1`.
- Above the limit: output a Unit Delay of the sampled matrix, validity `0`.
- The Unit Delay has initial condition `eye(3,3)` and inherited sample time `-1` (from the sampled input).

**Important distinction:** the annotation says “last valid measure,” but the delay input is the raw sampler output, not a conditionally accepted measurement. It therefore stores the **previous sample**, even if that sample was also invalid. During prolonged invalidity, this is not a true last-valid-value hold. The slew-rate/validity paths are not explicitly sampled by the same 1 Hz hold, so synchronized acquisition/validity semantics remain to be designed.

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

## 6. Known issues / next engineering steps

These are findings from source/wiring inspection, **not newly fixed in this documentation update**:

1. **Misalignment is added as a full near-identity matrix.** With zero misalignment angles and zero noise, the saved expression becomes `A_meas = A_true + I`, not `A_true`. Fix the intended composition before treating the output as a valid DCM.
2. **First-order error matrices are not exact rotations.** Even after correcting additive composition, `I + skew(error)` generally is only approximately orthogonal. Decide on a consistent exact rotation/quaternion composition, or explicitly quantify the small-angle approximation. Do not add independent errors to nine DCM elements.
3. **Axis/sign/frame conventions need reconciliation.** The script says 312 misalignment, the constructor has its own element pattern, and fi/psi noise paths are crossed. Establish sensor axes, boresight/roll axis, body-to-sensor transform, error frame and multiplication side before changing signs or channel mapping.
4. **Accuracy scaling is not yet justified.** Verify `/3` from a 3-sigma specification and the additional `/sqrt(2)` against the chosen noise definition. Confirm independence of random streams; all three currently specify seed `0`.
5. **Invalid measurement handling is previous-sample, not last-valid hold.** Define the desired acquisition-time behaviour, retain the last accepted measurement, and keep validity false for invalid acquisitions. Define startup behaviour if the first acquisition is invalid.
6. **Validity is only a slew-rate criterion.** There is no implemented star-identification/optics model, field of view, Sun/Earth exclusion, processing latency or general dropout model. Do not describe the current sensor as a full star-tracker simulator.
7. **Estimator and gyro interfaces remain unfinished.** Route measurement and validity together before adding attitude determination or feedback; never substitute noisy sensor attitude for truth in environmental calculations.
8. **Workspace initialization remains manual when using Simulink Run.** Consider a separate parameter-only initializer only with explicit approval.

Recommended validation, in order:
- Zero-noise/zero-misalignment test: measured attitude equals truth.
- Rotation checks: `A*A' ≈ I`, `det(A) ≈ +1`.
- Known single-axis perturbations: verify signs, axis assignment and composition order.
- Long fixed-attitude run: measure angular-error mean, standard deviation, cross-correlation and repeatability against the requested sensor statistics.
- Rate sweep below, at and above the limit; prolonged invalid interval; first sample invalid; recovery.
- Confirm 1 Hz acquisition timestamps, held output and synchronized validity.
- Only then integrate estimator/control and test against independently logged truth.

## 7. Working conventions for another AI assistant

- The user wants to learn the engineering as it is built. Explain actual block connections, equations, frames and units; avoid silently adding architecture beyond the request.
- **Questions, reviews and explanations are read-only. Obtain permission before editing files/models.** This README is context, not standing authorization to modify the project.
- Inspect the active branch, Git status and actual saved model first. Do not import conclusions from another branch or assume old block SIDs remain valid.
- Save/close desktop models before branch changes. Uncommitted edits can follow branch switches; commits define branch snapshots. Never save a stale tool-session model over newer desktop edits.
- Use `main.m` as the current entry point. Do not revive `project.m` or old `project_lib` references.
- Distinguish intended behaviour, inspected implementation, structural checks and executed numerical tests. A successful compile is not proof of correct sensor statistics or physics.
- Preserve user wiring and make small scoped edits. Back up SLX files before multi-step changes. Do not commit, push, merge or reset branches without authorization.
- SLX files are tracked with **Git LFS** (`*.slx filter=lfs`). Install/initialize LFS before checkout/commit; otherwise apparent binary modifications or pointer-only files can mislead. `git lfs fsck` checks local object integrity.
- If using the local MATLAB MCP tools, load referenced library definitions to inspect internals; `model_read` may expose only linked interfaces. Resolve exact paths/SIDs rather than guessing names containing `/`.

## 8. Mission references

- [ESA: GOMX-4B mission overview](https://www.esa.int/Enabling_Support/Space_Engineering_Technology/Technology_CubeSats/GOMX-4B).
- Holst et al., [*Attitude and Orbit Control Results of the GOMX-4 Tandem CubeSat Mission*](https://davidgerhardt.com/publications/GOMX4_IAC2018.pdf), IAC-18-B4.6B.11, 2018.

These sources motivate the mission concept; they do not validate the current geometry, inertia, sensor accuracy or calibration assumptions.
