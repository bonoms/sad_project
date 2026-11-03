# AURA — 6U CubeSat Attitude Dynamics

University spacecraft attitude dynamics project developed in **MATLAB and Simulink**. The spacecraft concept is inspired by **GOMX-4B**, a real 6U CubeSat technology-demonstration mission involving Earth observation and inter-satellite communications. This project is an educational adaptation, not a reconstruction of its flight configuration or telemetry.

## Mission and current scope

The reference scenario is a **near-polar, approximately Sun-synchronous low Earth orbit**. The current model studies the **uncontrolled attitude response to environmental disturbances**, starting from rest. Magnetic-coil actuation and star-tracker-based attitude determination are intended extensions; neither is implemented as a closed-loop subsystem yet.

Implemented so far:
- Keplerian orbit propagation with nonzero eccentricity.
- Nonlinear rigid-body rotational dynamics.
- Switched Euler 3-1-2 / 3-1-3 attitude kinematics, converted to an inertial-to-body direction cosine matrix (DCM).
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
| Orbit size parameter | `height = 449 km` | Defined as `sma − R_E`; not constant instantaneous altitude because eccentricity is nonzero |
| Semi-major axis | `sma = R_E + height` | Derived orbit parameter |
| Eccentricity | `ecc = 0.002` | Prescribed orbit parameter |
| Inclination | `inc = deg2rad(97.32)` | Prescribed near-polar orbit parameter |
| Initial true anomaly | `ta0 = 0 rad` | Starts at perigee |
| Initial angular velocity | `[0, 0, 0] rad/s` | Body relative to inertial space, expressed in body axes |
| Initial attitude | Identity DCM, implied by `e0` | Body axes initially aligned with inertial axes; `A0` is retained in the script but is not the active initialization input |
| Initial Euler angles | `e0 = [0; 0; 0] rad` | Initial attitude for the active switched Euler kinematics |
| Residual magnetic dipole | `j_B = [0.01, 0.05, 0.01] A·m²` | Assumed constant body-frame disturbance dipole; **not electric current or a commanded actuator dipole** |

Although `omegax0`, `omegay0` and `omegaz0` are each assigned 1°/s, the assembled `omega0` vector is multiplied by zero. Therefore, the current experiment starts at rest; it does **not** use an Ariane 6 release-rate scenario or test detumbling.

### Environmental constants

| Parameter | Value |
|---|---|
| Earth gravitational parameter | `mu_E = 398600.433 km³/s²`, from `astroConstants(13)` |
| Earth reference radius | `R_E = 6371.01 km`, from `astroConstants(23)` |
| Earth rotation rate | `omega_E ≈ 7.29210659088e-5 rad/s`, derived from `astroConstants(53)` |
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

The linked Sun-direction block has its own hardcoded annual period (`31558149.76 s`) and obliquity (`0.4091 rad`). The script declares `T_sun` and `eps_E`, but changing those declarations alone does not change that linked block.

## Running the simulation

Download or clone the **complete repository** to any location. MATLAB and Simulink are required (tested with R2026b); no personal paths or external coursework folders are needed.

```text
sad/
├── main.m
├── aura_mission.slx
└── library/
    ├── library.slx
    ├── astroConstants.m
    └── slblocks.m
```

Open `main.m` in MATLAB and click **Run**. It adds the repository root and `library/` to the MATLAB path, loads `library/library.slx` using the script's absolute location, initializes the workspace, runs `aura_mission.slx`, and animates the body axes. No personal paths or external library files are required. Keep the directory structure above intact when moving or cloning the repository.

`main.m` is the sole simulation entry point; the obsolete `project.m` script has been removed.

### Work in progress on this branch

`bug/division-by-zero-313` preserves the experimental enabled-subsystem implementation in `Euler 312/313 Kinematics1`, including sampled mode selection. Initial-condition handover and the selected attitude output remain unfinished. This checkpoint is not a claim that the experimental subsystem is simulation-ready.

- **Solver:** fixed-step fourth-order Runge–Kutta (`ode4`).
- **Step:** 0.1 s.
- **Requested duration:** three orbital periods (`stopTime = 3*T`), with `T = 2*pi/sqrt(mu_E/sma^3)`. The final logged time follows the 0.1 s fixed-step grid.
- **Outputs:** `out.tout` (time in seconds), `out.A_BN` (3×3×N attitude DCM array), and `out.omega` (3×1×N body angular velocity array, rad/s). The summed disturbance torque is not currently exported as a separate output.
- **Animation:** red, green, and blue arrows show the body X, Y, and Z axes; every 100th logged sample is displayed.

### Library loading and maintenance

The model's custom block links reference `library/...`, matching `library/library.slx`. If MATLAB reports a missing `project_lib`, the model contains stale links from the old library name; loading `library.slx` alone does not repair those references. Use the updated model and library together.

`library/slblocks.m` registers the library as **SAD Project** in the Simulink Library Browser. Browser registration is optional for simulation. To refresh it after running the initialization, use:

```matlab
sl_refresh_customizations;
```

The Euler-313 kinematics block guards its denominators while its integrator is held in reset (the inactive branch). This avoids division-by-zero warnings at the zero-angle initial attitude without changing the active kinematics equations. It does not remove the mathematical singularities of Euler angles in general.

## Mission references

- [ESA: GOMX-4B mission overview](https://www.esa.int/Enabling_Support/Space_Engineering_Technology/Technology_CubeSats/GOMX-4B).
- Holst et al., [*Attitude and Orbit Control Results of the GOMX-4 Tandem CubeSat Mission*](https://davidgerhardt.com/publications/GOMX4_IAC2018.pdf), IAC-18-B4.6B.11, 2018.

These sources motivate the reference mission and hardware concept. The current numerical assumptions should not be attributed to the real spacecraft unless separately verified.
