# AURA — 6U CubeSat Attitude Dynamics

University spacecraft attitude dynamics project developed in **MATLAB and Simulink**. The spacecraft concept is inspired by **GOMX-4B**, a real 6U CubeSat technology-demonstration mission involving Earth observation and inter-satellite communications. This project is an educational adaptation, not a reconstruction of its flight configuration or telemetry.

## Mission and current scope

The reference scenario is a **near-polar, approximately Sun-synchronous low Earth orbit**. The current model studies the **uncontrolled attitude response to environmental disturbances**, starting from rest. Magnetic-coil actuation and star-tracker-based attitude determination are intended extensions; neither is implemented as a closed-loop subsystem yet.

Implemented so far:
- Keplerian orbit propagation with nonzero eccentricity.
- Nonlinear rigid-body rotational dynamics.
- Inertial-to-body direction cosine matrix (DCM) propagation.
- Gravity-gradient disturbance torque.
- Residual magnetic-dipole torque using a rotating, tilted Earth-dipole field model.
- Six-face solar radiation pressure (SRP), including absorption, specular and diffuse reflection.
- Attitude logging and a 3D animation of the body axes.

## Current parameters and initial conditions

Values below describe the **current saved simulation**, not verified GOMX-4B flight data.

| Parameter | Current value | Type / interpretation |
|---|---|---|
| Spacecraft class | 6U CubeSat | Mission concept |
| Reference mass | Approximately 8 kg | Background mission reference; not an explicit input to the current rotational model |
| Inertia matrix | `diag([0.087, 0.067, 0.033]) kg·m²` | Assumed diagonal rigid-body inertia |
| Orbit size parameter | `h = 449 km` | Defined as `a − R_E`; not constant instantaneous altitude because eccentricity is nonzero |
| Semi-major axis | `a = R_E + h` | Derived orbit parameter |
| Eccentricity | `e = 0.002` | Prescribed orbit parameter |
| Inclination | `i = 97.32°` | Prescribed near-polar orbit parameter |
| Initial true anomaly | `theta_0 = 0 rad` | Starts at perigee |
| Initial angular velocity | `[0, 0, 0] rad/s` | Body relative to inertial space, expressed in body axes |
| Initial attitude | `A0 = eye(3)` | Body axes initially aligned with inertial axes |
| Initial Euler angles | `e0 = [0; 0; 0] rad` | Retained initialization for the disabled Euler path |
| Residual magnetic dipole | `j_b = [0.01, 0.05, 0.01] A·m²` | Assumed constant body-frame disturbance dipole; **not electric current or a commanded actuator dipole** |

Although `wx_0`, `wy_0` and `wz_0` are each assigned 1°/s, the assembled `w_0` vector is multiplied by zero. Therefore, the current experiment starts at rest; it does **not** use an Ariane 6 release-rate scenario or test detumbling.

### Environmental constants

| Parameter | Value |
|---|---|
| Earth gravitational parameter | `mu_E = 398600.4418 km³/s²` |
| Earth reference radius | `R_E = 6371 km` |
| Earth rotation rate | `w_E = 7.292115e-5 rad/s` |
| Magnetic-field scale in the model | `H_0 = 2.98059e-5 T` |
| Magnetic-axis tilt in the model | `0.200713 rad` |
| Solar irradiance in the SRP block | `1358 W/m²` |

Orbital distances use **km**, while inertia, face geometry, magnetic dipole and torques use **SI units**. Torque outputs are in **N·m**. The DCM maps inertial-frame column vectors into body-frame components.

### SRP face parameters

Face order is **+X, −X, +Y, −Y, +Z, −Z**. Face normals are the corresponding signed body-axis unit vectors. These geometry and optical values are preliminary assumptions, not a verified spacecraft CAD model.

| Face parameter | Current values, in face order |
|---|---|
| Area [m²] | `[0.02, 0.02, 0, 0.03, 0.06, 0.06]` |
| Specular coefficient | `[0.15, 0.5, 0.5, 0.5, 0.5, 0.5]` |
| Diffuse coefficient | `[0.05, 0.05, 0.05, 0.05, 0.05, 0.05]` |

Body-frame force application points, relative to the assumed centre of mass, are stored as columns of `r_F` in metres:

```matlab
r_F = [0.05 -0.25  0     0     0     0;
       0     0     0.10 -0.10  0     0;
       0     0     0     0     0.05 -0.05];
```

The linked Sun-direction block has its own hardcoded annual period and obliquity. The script also declares `T_sun`, `r_sun` and `epsilon`, but changing those declarations alone does not change that linked block.

## Running the simulation

Download or clone the **complete repository** to any location. MATLAB and Simulink are required (tested with R2026b); no personal paths or external coursework folders are needed.

```text
sad/
├── project.m
├── aura_mission.slx
└── library/
    ├── laythe.slx
    └── SAD_Library.slx
```

Open `project.m` in MATLAB and click **Run**. Its first lines add the repository and its subfolders to the MATLAB path, then load the bundled libraries. No helper script or personal path is required; the complete repository can be moved or renamed without editing the code. Keep `aura_mission.slx` alongside `project.m` and the libraries in a repository subfolder.

- **Solver:** fixed-step fourth-order Runge–Kutta (`ode4`).
- **Step:** 0.1 s.
- **Requested duration:** ten orbital periods, with `T = 2*pi/sqrt(mu_E/a^3)`.
- **Main outputs:** `out.Ab_n` (attitude DCM), `out.w` (body angular velocity) and `out.dist_torque` (summed disturbance torque).

## Verification completed

The dynamics, orbit, coordinate transformations and disturbance blocks have been checked using analytical reference cases and real MATLAB/Simulink execution. Checks include torque-free energy/angular-momentum conservation, constant-torque response, known-axis rotations, orbital radius consistency and independent torque-formula comparisons.

An Euler 312/313 switching implementation produced singularity warnings and timestep-dependent attitude errors. The active simulation now uses the existing **DCM propagation block** instead; the Euler propagation/conversion path is commented out.

The updated entry script was tested in **MATLAB/Simulink R2026b** over approximately ten orbits: **560,516 samples**, final time **56,051.5 s**, finite attitude and angular-rate outputs, and no execution warnings. Separate DCM tests confirmed orthogonality and timestep convergence. These checks establish numerical consistency for the tested cases, not flight-model accuracy.

## Remaining work

- Add aerodynamic disturbance torque.
- Add Earth-shadow/eclipse gating to direct SRP.
- Refine inertia, face geometry, pressure-centre offsets and residual dipole assumptions.
- Add magnetic actuator limits, sensor models, attitude estimation and detumbling/pointing control.
- Test nonzero deployment rates and a wider range of initial attitudes and environmental conditions.

## Mission references

- [ESA: GOMX-4B mission overview](https://www.esa.int/Enabling_Support/Space_Engineering_Technology/Technology_CubeSats/GOMX-4B).
- Holst et al., [*Attitude and Orbit Control Results of the GOMX-4 Tandem CubeSat Mission*](https://davidgerhardt.com/publications/GOMX4_IAC2018.pdf), IAC-18-B4.6B.11, 2018.

These sources motivate the reference mission and hardware concept. The current numerical assumptions should not be attributed to the real spacecraft unless separately verified.
