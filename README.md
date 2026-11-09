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

## 7. Model walkthrough and exam study guide

This section explains the active mission model from physical idea to signal wiring. It covers the substantial subsystems rather than elementary helpers such as Vector Norm. Equations describe the inspected implementation and its conventions; the numerical limitations documented above still apply.

### 7.1 The complete model in one explanation

The model asks: **how does an uncontrolled rigid spacecraft rotate under environmental torques, and what attitude would an imperfect star tracker report?** It separates four tasks:

1. **Orbital geometry:** determine where the spacecraft is relative to Earth and the local orbital frame.
2. **Environment:** calculate external torques in body coordinates from gravity gradient, the magnetic field and sunlight.
3. **Dynamics and kinematics:** integrate torque into angular velocity, then angular velocity into orientation.
4. **Measurement:** perturb and sample the true orientation without changing the underlying physical motion.

```text
main.m: physical constants, orbit, inertia, geometry, initial conditions, sensor parameters
                                      |
                                      v
Environment
  Orbit Propagator ── r ────────────┬──> Gravity Gradient Torque ──┐
         |                         └──> Magnetic Field Torque ────┤
         └── true anomaly ──> LVLH ───> Gravity Gradient Torque   ├──> Sum: M_B
  Sun Direction + face geometry ──────> SRP Torque ───────────────┘       |
         ^                                                             v
         |                                                          dynamics
         |                                                             |
         |                                                             v
         |                                                           omega_B
         |                                                             |
         |                                                   Euler 312/313 Kinematics
         |                                                             |
         |                                                   output in 312 coordinates
         |                                                             |
         └──────────────── A_BN <────────────────────── Euler 312 to DCM
                              |
                              ├──> truth logging and animation
                              └──> star_tracker <── omega_B
                                        |
                               measurement + validity logging
```

The feedback from attitude to environment is **physical coupling, not attitude control**. For example, rotating the spacecraft changes which faces receive sunlight and how the magnetic field projects into body axes. Those changes modify torque and therefore subsequent motion. There is no desired-attitude reference, control error or commanded actuator torque in this loop.

The integrators store the state, so the drawing is not simply an instantaneous circular calculation. At initialization, their initial conditions establish the starting orbit phase, angular velocity and attitude. During continuous integration the solver evaluates the coupled equations, including intermediate stages of `ode4`; the diagram should not be interpreted as every subsystem executing only once in a fixed left-to-right order per step.

**A concise oral explanation:** “The orbit supplies environmental geometry. The environment produces body-frame disturbances. Euler's rigid-body equation gives angular acceleration, integrated into body rate. Euler-angle kinematics gives attitude, fed back to the environment. Separately, the sensor converts truth into noisy sampled measurements.”

### 7.2 Frames, state variables and why dynamics is not kinematics

| Quantity | Meaning | Role in the model |
|---|---|---|
| `r_N` | Earth-centred position expressed in inertial coordinates | Orbit output; used by environmental models |
| `A_LN` | Maps inertial components to local orbital components | Supplies the radial direction for gravity gradient |
| `A_BN` | Maps inertial components to body components | Common physical-attitude interface |
| `omega_B = [w1; w2; w3]` | Body-relative-to-inertial angular velocity, expressed in the body frame | Integrated rotational dynamic state |
| `e = [phi; theta; psi]` | Euler coordinates of orientation in a specified sequence | Integrated kinematic coordinates, not body-rate components |
| `M_B` | Total external torque expressed in the body frame | Input to rotational dynamics |

**Dynamics answers why the angular velocity changes; kinematics answers how the orientation changes given that angular velocity.** In general, `dot(e)` is not equal to `omega_B`: the relation depends on the current Euler angles and sequence.

For an ideal DCM, `A_NB = A_BN.'`. A vector is transformed by multiplication, not by adding Euler angles. Matrix order matters: `A_BN * A_LN.'` maps local-orbit components to body components. Matrix rows of `A_BN` give the body axes expressed in inertial coordinates, explaining why `main.m` uses rows for its body-axis animation.

The first, second and third coordinate axes have consistent array positions throughout the equations. Their physical identification with spacecraft hardware or tracker boresight requires a separate convention; the words roll, pitch and yaw do not supply that convention by themselves.

### 7.3 Orbit Propagator — advance orbital phase, then reconstruct position

**Location:** `Environment/Orbit Propagator`, linked to `library/Orbit Propagator`.

**Inputs:** semimajor axis `a = sma`, eccentricity `e = ecc`, inclination `i = inc`, initial true anomaly `nu0 = ta0`, and gravitational parameter `mu = mu_E`. **Outputs:** inertial position vector `r_N` and true anomaly `nu`.

**Idea.** In a two-body ellipse, orbital elements are constant but true anomaly does not advance uniformly. The spacecraft moves faster near perigee and slower near apogee. Instead of integrating all six Cartesian position/velocity components, this block integrates one scalar phase and reconstructs the position geometrically.

The implemented equations are

$$
n=\sqrt{\frac{\mu}{a^3}},\qquad
\dot\nu=\frac{n(1+e\cos\nu)^2}{(1-e^2)^{3/2}},\qquad
r=\frac{a(1-e^2)}{1+e\cos\nu},
$$

$$
\mathbf r_N=r\begin{bmatrix}\cos\nu\\\sin\nu\cos i\\\sin\nu\sin i\end{bmatrix}.
$$

**How the wiring implements it:**

1. `mu/a^3` and a square root produce mean motion `n`.
2. Feedback from the true-anomaly Integrator supplies `cos(nu)`.
3. Products, sums and powers form the nonlinear `dot(nu)` expression.
4. The Integrator starts at `ta0` and advances true anomaly.
5. A separate algebraic branch forms radius magnitude and the inclined unit-position vector, then multiplies them.
6. Position branches to gravity-gradient and magnetic torque; true anomaly branches to the LVLH frame block.

The phase equation follows from conserved orbital angular momentum: `dot(nu) = h/r^2`, with `h = sqrt(mu*a*(1-e^2))`. This explains the squared factor in the numerator and why replacing it by constant `n` would be wrong for a noncircular orbit.

**Assumptions to explain:** this active block has no RAAN or argument-of-perigee inputs; its position formula corresponds to those orientation angles being zero. It does not propagate J2 precession, drag, or orbit changes caused by disturbances. The near-Sun-synchronous inclination is an initial scenario choice, not a simulated Sun-synchronous precession mechanism. For nonzero eccentricity, its transverse orbital direction is not generally the exact velocity direction.

### 7.4 LVLH Reference Frame — construct a moving orbital basis

**Location:** `Environment/LVLH Reference Frame`, linked to `library/LVLH Reference Frame`.

**Inputs:** inclination and true anomaly. **Output:** `A_LN`.

**Idea.** Gravity gradient needs the Earth-radial direction expressed in the body, not just the distance from Earth. This block describes an orbital basis whose first axis points radially outward, second axis points transverse in the direction of increasing true anomaly, and third axis is the orbit normal. These are the conventions of this implementation; other definitions called LVLH can use different axis orders and signs.

The matrix assembled from its rows is

$$
A_{LN}=\begin{bmatrix}
\cos\nu&\sin\nu\cos i&\sin\nu\sin i\\
-\sin\nu&\cos\nu\cos i&\cos\nu\sin i\\
0&-\sin i&\cos i
\end{bmatrix}.
$$

Sine/cosine blocks generate the shared terms. Products generate the mixed terms; Mux/Reshape blocks form rows; concatenation along dimension 1 assembles the matrix.

The radial unit vector has local components `[1;0;0]`. Therefore

$$
\hat{\mathbf r}_B=A_{BN}A_{LN}^{T}\begin{bmatrix}1\\0\\0\end{bmatrix}.
$$

This is the actual frame-conversion chain used by the gravity-gradient subsystem. It is not a change to the spacecraft attitude state: it is another basis used to calculate a torque.

### 7.5 Gravity Gradient Torque — differential gravity acting on an extended body

**Location:** `Environment/Gravity Gradient Torque`, linked to `library/Gravity Gradient Torque`.

**Inputs:** position, gravitational parameter, inertia, local-orbit DCM and body-attitude DCM. **Output:** gravity-gradient torque in body coordinates.

**Idea.** The nearer and farther parts of the spacecraft experience slightly different gravitational acceleration. The resulting moment depends on the mass distribution and orientation relative to the Earth-radial direction. A point-mass orbital model alone cannot describe this rotational effect.

$$
\mathbf M_{GG,B}=\frac{3\mu}{r^3}\left[\hat{\mathbf r}_B\times(I\hat{\mathbf r}_B)\right].
$$

**Signal path:**

1. Transpose `A_LN`, then multiply by `A_BN`.
2. Apply that transformation to local `[1;0;0]` to obtain `rhat_B`.
3. Multiply `rhat_B` by inertia.
4. Compute `3*mu/r^3` from the orbit radius.
5. The Cross Product receives the scaled radial vector first and `I*rhat_B` second, giving the equation above.

The cross product vanishes when the radial direction is a principal inertia axis, or for spherical inertia `I = J*eye(3)`. This does not mean that every principal-axis radial orientation is dynamically stable; stability is a separate question. The orbit supplies radius and radial direction, while the attitude supplies how that direction appears to the spacecraft.

**Units:** `mu_E` is in km³/s² and `r` is in km, so `mu_E/r^3` has units s⁻². Multiplication by SI inertia gives N·m; converting only one of `mu_E` or `r` would break consistency.

### 7.6 Magnetic Field Torque — residual dipole in a rotating Earth field

**Location:** `Environment/Magnetic Field Torque`, linked to `library/Magnetic Field Torque`.

**Inputs:** inertial position, residual body dipole `j_B`, and true attitude. **Output:** magnetic torque in the body frame.

**Idea.** A spacecraft can have a permanent or residual magnetic dipole even with no active magnetorquer. That dipole experiences a torque in Earth's field. The model approximates Earth by a tilted dipole rotating with Earth.

Let `alpha` be the dipole tilt and `omega_E` the hardcoded Earth rotation rate. The Clock drives the inertial dipole-axis direction

$$
\hat{\mathbf m}_N(t)=\begin{bmatrix}
\sin\alpha\cos(\omega_Et)\\
\sin\alpha\sin(\omega_Et)\\
\cos\alpha
\end{bmatrix}.
$$

The field and torque calculations are

$$
\hat{\mathbf r}_N=\frac{\mathbf r_N}{\|\mathbf r_N\|},\qquad
\mathbf B_N=H_0\left(\frac{R_{eq}}{r}\right)^3
\left[3(\hat{\mathbf m}_N\cdot\hat{\mathbf r}_N)\hat{\mathbf r}_N-\hat{\mathbf m}_N\right],
$$

$$
\mathbf B_B=A_{BN}\mathbf B_N,\qquad
\mathbf M_{mag,B}=\mathbf j_B\times\mathbf B_B.
$$

**Wiring logic:** normalize position; take its dot product with the rotating dipole axis; multiply the result by three and the radial unit vector; subtract the dipole axis; scale by `H_0*R_eq^3/r^3`; transform to body coordinates; finally take the dipole–field cross product in that order.

The block hardcodes `H_0 = 2.98059e-5`, `R_eq = 6.37813e3 km`, tilt `2.00712e-1 rad` and rotation rate `7.29210e-5 rad/s`. The symbol `H_0` is used as the field scale in this torque model; the dipole cross product requires magnetic flux density in tesla. The workspace constants with similar names do not automatically replace these hardcoded values.

Torque vanishes for a dipole parallel to the field and is largest when perpendicular. Here `j_B` is a disturbance parameter, not a commanded coil current or implemented magnetic controller. The field is a simplified dipole model rather than a high-order geomagnetic model with date and geolocation inputs.

### 7.7 Sun Direction and Solar Radiation Pressure — illumination, force, moment

**Locations:** `library/Sun Direction` and `Environment/Solar Radiation Pressure Torque`, linked to `library/Solar Radiation Pressure`.

**Idea.** Photons carry momentum. Absorbing and reflecting sunlight create forces on exposed faces. If a resultant force acts away from the centre of mass, it creates torque. The force depends on attitude because face illumination and reflection geometry change as the spacecraft rotates.

The Sun Direction block uses simulation time to construct

$$
\lambda=\frac{2\pi t}{31558149.76},\qquad
\hat{\mathbf s}_N=\begin{bmatrix}\cos\lambda\\\cos(0.4091)\sin\lambda\\\sin(0.4091)\sin\lambda\end{bmatrix}.
$$

This is a unit direction towards the Sun with fixed obliquity and uniform annual phase. It is not an ephemeris tied to a real launch date. The body direction is `s_B = A_BN*s_N`.

For face `k`, define outward unit normal `n_k`, area `A_k`, lever arm `r_F,k`, specular coefficient `rho_s,k` and diffuse coefficient `rho_d,k`. The implemented force is

$$
u_k=\mathbf n_k\cdot\hat{\mathbf s}_B,\qquad
q_k=\frac{1358}{299792458}\,A_k\max(u_k,0),
$$

$$
\mathbf F_{k,B}=-q_k\left[(1-\rho_{s,k})\hat{\mathbf s}_B+
\left(2\rho_{s,k}u_k+\frac{2}{3}\rho_{d,k}\right)\mathbf n_k\right],
\qquad
\mathbf M_{SRP,B}=\sum_k\mathbf r_{F,k}\times\mathbf F_{k,B}.
$$

Here `u_k` is the incidence cosine; `q_k` has units of force. The leading minus sign is appropriate because `s_B` points towards the Sun while incident photon momentum points away from it. Specular reflection changes momentum normal to the face; the diffuse term uses the implemented `2/3` factor. The coefficient on the Sun-direction term includes both absorption and the incident momentum associated with diffuse reflection.

**How the subsystem is organized:**

1. Reshape the area and optical-coefficient arrays into face-indexed data.
2. The **For Each Subsystem** partitions inputs 1–5 by columns: one normal, optical-coefficient pair, area and lever arm per iteration. The attitude input is shared, not partitioned.
3. Each iteration transforms the Sun direction and calculates incidence. A Switch outputs the incidence cosine only when positive, otherwise zero, so back-facing surfaces contribute no force.
4. Products and sums form the absorption/reflection force; Gain `-1` sets its sign.
5. Each face computes `cross(r_F,k, F_k)`.
6. The outer Sum of Elements collapses dimension 2, summing face-torque columns into one three-component torque.

An illuminated face can exert force but no moment if the lever arm is zero or parallel to that force. Setting a face area to zero removes its SRP contribution without removing the face from the arrays. Current assumptions include no eclipse, no self-shadowing, constant irradiance and fixed optical properties.

### 7.8 Rotational dynamics — Euler's equation in body coordinates

**Location:** `aura_mission/dynamics`. This is the active dynamics implementation; it is not simply the unused `library/Dynamics` block.

**Inputs:** `omega0`, `I`, `I_inv` and total body torque `M`. **Output:** true body angular velocity `omega`.

Angular momentum is `H_B = I*omega_B`. Because the body basis itself rotates, the torque is not merely `I*dot(omega_B)`:

$$
\mathbf M_B=I\dot{\boldsymbol\omega}_B+\boldsymbol\omega_B\times(I\boldsymbol\omega_B),
\qquad
\dot{\boldsymbol\omega}_B=I^{-1}\left[\mathbf M_B-\boldsymbol\omega_B\times(I\boldsymbol\omega_B)\right].
$$

**Actual wiring and its sign:**

- Integrator output `omega` feeds `Matrix Multiply1`, producing `I*omega`.
- The Cross Product receives **`I*omega` first and `omega` second**, so it outputs `(I*omega) × omega`.
- The Sum adds this to `M`. This is correct because `(I*omega) × omega = -omega × (I*omega)`.
- `Matrix Multiply2` applies `I_inv` to produce angular acceleration.
- The Integrator starts from its external initial condition `omega0`; its output feeds kinematics, sensors, logging and the dynamic feedback calculation.

Even with zero torque, a nonspherical rigid body can have changing body-frame angular-velocity components because of gyroscopic coupling. This does not imply changing inertial angular momentum. For spherical inertia the cross term vanishes, making constant torque-free body rate a useful verification case. The model assumes a rigid body with constant inertia and no internal momentum devices.

### 7.9 Euler kinematics and DCM conversion — representing the same attitude

**Locations:** `Euler 312/313 Kinematics`, its linked `Euler 312 Kinematics` and `Euler 313 Kinematics`, and the Euler/DCM conversion blocks.

Use `e = [phi; theta; psi]` and body-rate components `[w1; w2; w3]`. Sequence 312 means successive rotations about axes 3, 1 and 2; sequence 313 repeats axis 3 after the axis-1 rotation. These are coordinate parameterizations, not three separate physical spacecraft motions that are simulated independently.

#### Rate equations and integrators

The configured 312 arithmetic implements

$$
\dot\phi=\frac{w_3\cos\psi-w_1\sin\psi}{\cos\theta},\qquad
\dot\theta=w_1\cos\psi+w_3\sin\psi,
$$
$$
\dot\psi=w_2-\tan\theta\,(w_3\cos\psi-w_1\sin\psi).
$$

The configured 313 arithmetic, when its denominator guards are not selected, implements

$$
\dot\phi=\frac{w_1\sin\psi+w_2\cos\psi}{\sin\theta},\qquad
\dot\theta=w_1\cos\psi-w_2\sin\psi,
$$
$$
\dot\psi=w_3-\frac{w_1\sin\psi+w_2\cos\psi}{\tan\theta}.
$$

Each block separates rates and angles with Demux blocks, forms the trigonometric combinations, gathers the three derivatives in a Mux, and integrates them. Its normal Integrator output provides both angle feedback and exported Euler angles.

The formulas become singular when the coordinate mapping loses rank: for 312 at `cos(theta)=0`; for 313 at `sin(theta)=0`. This is a failure of the coordinates, not an infinite physical angular velocity. The 313 cotangent term should be understood through its limiting behaviour as well as the explicit block arithmetic.

Some block labels do not match their configured operation: for example, `Cos2` is configured as `sin`, the 312 `Cos3` as `tan`, and `DCM to Euler 313/Asin` as `acos`. The equations above follow configuration and wiring, not those labels.

#### Euler angles to DCM

The converters are algebraic: they evaluate sine/cosine products, assemble nine entries, and Reshape the result into a 3×3 matrix using MATLAB column-major ordering. They add no integration state.

With passive elementary rotations

$$
R_1(a)=\begin{bmatrix}1&0&0\\0&\cos a&\sin a\\0&-\sin a&\cos a\end{bmatrix},\quad
R_2(a)=\begin{bmatrix}\cos a&0&-\sin a\\0&1&0\\\sin a&0&\cos a\end{bmatrix},\quad
R_3(a)=\begin{bmatrix}\cos a&\sin a&0\\-\sin a&\cos a&0\\0&0&1\end{bmatrix},
$$

the implemented conventions are `A312 = R2(psi)*R1(theta)*R3(phi)` and `A313 = R3(psi)*R1(theta)*R3(phi)`. The rightmost rotation acts first. These formulas fix the signs and sequence more precisely than a generic phrase such as yaw–pitch–roll.

#### DCM to Euler angles

For matrix entries `Aij`, the saved converters extract

| Output | 312 converter | 313 converter |
|---|---|---|
| `phi` | `atan2(-A21, A22)` | `atan2(A31, -A32)` |
| `theta` | `asin(A23)` | `acos(A33)` |
| `psi` | `atan2(-A13, A33)` | `atan2(A13, A23)` |

The Reshape/Demux chain reads matrix entries in column-major order. `atan2` retains quadrant information that a simple ratio and `atan` would lose. Inverse trigonometric functions return a principal branch: different angle triples can represent the same DCM, and angle wrapping alone is not an attitude discontinuity. At the relevant singularities, individual Euler angles are not uniquely recoverable.

### 7.10 Euler 312/313 switching — coordinate handover and reset logic

**Idea.** Use a different Euler chart before the current chart approaches a singularity. The attitude must stay the same when the coordinates change; the incoming chart needs angles derived from the outgoing physical attitude, not copied components or a restart at the original initial attitude.

**The actual saved architecture uses resettable integrators, not Enabled Subsystems.** The mode signal is the output of `Relay1`:

| Relay mode | Selected output representation internally | 312 reset input | 313 reset input |
|---|---|---|---|
| `0` | 312 branch | `0` | `1` |
| `1` | 313 branch | `1` | `0` |

Both integrators are configured with **External reset = `level hold`**, external initial conditions, and a state port. A nonzero reset holds that integrator at its supplied initial condition; this is not equivalent to disabling all arithmetic in that branch. The 313 denominator guards substitute `1` while its reset input is positive, avoiding evaluation of its singular denominators in that inactive/reset-held state. This guard does not make active 313 coordinates globally nonsingular.

**Trace the two paths separately:**

1. **Mode selection:** `Selector` takes the second component of the 312 Integrator's exported state. `Abs` feeds the Relay. Its on threshold is **75°**, off threshold **70°**. Between thresholds the Relay retains its previous output. This hysteresis avoids rapid back-and-forth switching at one boundary. Selection is based on the 312 state, including its reset-held representation while 313 is selected—not on an independent 313 safety test.
2. **Initialization/handover path:** the state-port outputs of both Integrators are converted to their respective DCMs. `Switch` selects the DCM according to mode. `DCM to Euler 1` converts the selected DCM to 312 angles; `IC` supplies `e0` initially and then passes its input. Another 312-to-DCM conversion creates a common attitude from which both 312 and 313 external initial-condition inputs are calculated.
3. **Ordinary output path:** the normal angle outputs of both Integrators are converted to their respective DCMs. `Switch1` selects the physical attitude, and `DCM to Euler 2` exports it in **312 coordinates regardless of the internal mode**. The top-level 312-to-DCM block can therefore interpret the output consistently even while the 313 branch is selected.

The Integrator's extra **state port is not angular velocity and not a simple one-step delay**. It exposes the integration state without that step's reset effect, allowing reset-feedback calculations to use a state value without depending directly on the post-reset output. It can differ from the normal output during reset handling. This distinction is central to understanding the separate handover and ordinary-output paths.

**Important distinction for an exam:** the architecture's intention is continuous physical attitude with a change of coordinates; the constant-rate tests in Section 3 demonstrate that the current scheduling still accumulates propagation lag. Explain the intended handover and the measured limitation separately. Neither hysteresis nor valid DCM orthogonality proves correct handover timing.

### 7.11 Star tracker — a measurement model, not a star-identification algorithm

**Location:** `sensors/star_tracker`, including `skew_symmetrix_matrix`.

**Idea.** Start from the known true attitude and emulate angular measurement uncertainty, mounting misalignment, additive matrix bias, acquisition rate and rejection at excessive slew. The subsystem does not render a star field, identify stars, solve Wahba's problem, or estimate attitude from camera pixels.

**Follow the measurement path:**

1. Three Gaussian sources generate unit-variance scalar noise; gains apply the angular standard deviations listed in Section 4.
2. `skew_symmetrix_matrix` uses negative gains, Mux blocks for columns and matrix concatenation to construct `E = I - skew([theta; psi; fi])`. Although its name refers to skew symmetry, its output includes the identity; only `E-I` is skew-symmetric.
3. Matrix products form `A_mis_sT*A_BN*E`. This models angular errors through composition rather than adding an identity-like matrix to truth. Left- and right-multiplication are not interchangeable.
4. A Sum adds the separately declared `sT_bias`. This is an additive matrix error, not necessarily a pure angular bias or a stretch matrix; arbitrary choices can destroy rotation-matrix properties. It is currently zero.
5. `1Hz_sampler` captures and holds the candidate every second. The 0.1 s noise-source period and the 1 s acquisition period are separate design choices.
6. The output Switch compares true angular-speed magnitude against `max_slew_rate`. Excessive rate selects its **top input**, the stored value. Otherwise it selects its **bottom input**, the sampled candidate.
7. The selected output feeds the Unit Delay, preserving it during rejection. The parallel validity Switch outputs `0` for excessive rate and `1` otherwise. Identity is the fallback before any accepted measurement exists.

A concrete hold example: accept matrix A, then reject candidates B and C. Because feedback stores the **selected output**, the reported matrix remains A through both rejections. Delaying raw candidates instead would eventually report rejected B. Holding a matrix does not make it a current measurement: it becomes stale while the spacecraft continues rotating.

The small-angle approximation drops second-order rotation terms, so a candidate is only approximately orthogonal even with zero additive bias. Random error, fixed misalignment and additive matrix bias are different mechanisms. Also distinguish the 1 Hz sampled candidate, the reported held matrix, the instantaneous validity level and a fresh-acquisition event. The annotation acknowledges that finite camera-exposure validity is not modelled.

The sensor container logs the tracker output but does not yet supply an implemented estimator/controller. The gyro remains a placeholder and is not needed to explain the currently implemented tracker logic.

### 7.12 Simulation, outputs and how to defend the model

`main.m` initializes parameters, simulates for three nominal orbital periods with fixed-step fourth-order Runge–Kutta, and animates true attitude. The 0.1 s solver step controls continuous numerical integration; it does not redefine blocks with explicit sample periods. Held sensor values may be logged at successive solver timestamps even when the physical acquisition rate is slower. A long output array is therefore not evidence of an equally high measurement rate.

The animation uses the rows of `A_BN` as body-axis directions in inertial space, displaying every 100th sample. This display stride is a visualization choice, not a solver or sensor setting. The animation displays truth, not noisy tracker measurements.

A useful explanation order for an oral exam is:

1. State the physical question, uncontrolled rigid-body assumptions and absence of active control.
2. Define the frames, mapping direction of the DCM, units and initial conditions.
3. Trace orbit → environmental torques → angular velocity → attitude → environmental feedback.
4. Derive the dynamic cross-product sign and explain why Euler-angle rates differ from body rates.
5. Explain Euler singularities, the coordinate-switching intention and actual reset/state-port wiring.
6. Trace one environmental torque from physical mechanism to final body-frame moment.
7. Separate measurement generation from truth and explain noise, sampling, rejection and staleness.
8. Distinguish successful execution, matrix validity and agreement with independent physics. Cite the measured handover limitation rather than claiming a fully validated model.

The library also contains alternative DCM/quaternion kinematics, a separate Dynamics block and a more general Keplerian Orbit block. Their presence does **not** mean the mission runs these alternatives: the active paths are those traced above. Elementary helper blocks are best understood in the equations they support rather than as separate physical subsystems.

## 8. Mission references

- [ESA: GOMX-4B mission overview](https://www.esa.int/Enabling_Support/Space_Engineering_Technology/Technology_CubeSats/GOMX-4B).
- Holst et al., [*Attitude and Orbit Control Results of the GOMX-4 Tandem CubeSat Mission*](https://davidgerhardt.com/publications/GOMX4_IAC2018.pdf), IAC-18-B4.6B.11, 2018.

These sources motivate the mission concept; they do not validate the current geometry, inertia, sensor accuracy or calibration assumptions.
