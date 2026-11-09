%% Initialization

clear;
close all;
clc;

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot, 'library'));
load_system(fullfile(projectRoot, 'library', 'library.slx'));

%% Physical Constants

mu_E = astroConstants(13);
R_E = astroConstants(23);
f_E = astroConstants(43);
Re_E = 3 * R_E / (3 - f_E); 
omega_E = 2 * pi / (astroConstants(53) * 3600);
eps_E = deg2rad(astroConstants(63));
dipole_tilt_E = deg2rad(11.5);
T_sun = 365.256363 * 24 * 3600;

%% Spacecraft Orbital Parameters

% Keplerian elements
height = 449;
sma = R_E + height;
ecc = 0.002;
inc = deg2rad(97.32);
raan = 0;
aop = 0;

n = sqrt(mu_E / sma^3);
T = 2 * pi / n;

%% Spacecraft Physical Parameters

% Moment of inertia
Ix = 0.087;
Iy = 0.067;
Iz = 0.033; 
I = diag([Ix Iy Iz]);
I_inv = inv(I);

% Flowing current
j_B = [0.01 0.05 0.01];

% Panels normals
N_B = [1 0 0;
       -1 0 0;
       0 1 0;
       0 -1 0;
       0 0 1;
       0 0 -1]';

% Panel center points
r_F = [5 0 0; 
       -25 0 0;
       0 10 0;
       0 -10 0;
       0 0 5;
       0 0 -5]' * 1e-2; % m

% Panel areas
Area  = [2 2 0 3 6 6] * 1e-2; % m^2

% Specular reflection coefficients
rho_s = [0.15 0.5 0.5 0.5 0.5 0.5];

% Diffuse reflection coefficients
rho_d = [0.05 0.05 0.05 0.05 0.05 0.05];

% Sensor misalignment matrix star tracker

fi_mis_sT = 0.0000001; % given in 312 notation
theta_mis_sT = 0.0000001;
psi_mis_sT = 0.0000001;

A_mis_sT = [1 fi_mis_sT -theta_mis_sT; % this should act also as any bias
            -fi_mis_sT 1 psi_mis_sT;
            fi_mis_sT -psi_mis_sT 1];

% Sensor star tracker bias matrix

sT_bias = zeros(3,3); % we should define a pure streching matrix



% Star tracker accuracy and tolerated slew rate

fi_acc = deg2rad(0.02/3); % divided by 3 since it's reported as 3 sigma
theta_acc = deg2rad(0.02/3);
psi_acc = deg2rad(0.06/3); % defined as roll axis

max_slew_rate = deg2rad(0.3);

%% Initial conditions

% True anomaly
ta0 = 0;

% Angular velocity
omegax0 = deg2rad(1);
omegay0 = deg2rad(1);
omegaz0 = deg2rad(1);
omega0 = [omegax0; omegay0; omegaz0]*0;

% Attitude
e0 = [0 deg2rad(55) 0]';
A0 = eye(3,3); % not necessary but to make dcm block work 

%% Simulation

% Simulation parameters
stopTime = 10;
fixedStep = 0.001; % s

%% Simulink model
out = sim(fullfile(projectRoot, 'aura_mission.slx'), ...
    "StopTime", "stopTime", ...
    "SolverType", "Fixed-step", ...
    "Solver", "ode4", ...
    "FixedStep", "fixedStep", ...
    "ReturnWorkspaceOutputs", "on");

% Data extraction
N = length(out.tout);
ts = out.tout;
As_BN = out.A_BN;


%% Visualization

