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

%% Initial conditions

% True anomaly
ta0 = 0;

% Angular velocity
omegax0 = deg2rad(5);
omegay0 = deg2rad(10);
omegaz0 = deg2rad(5);
omega0 = [omegax0; omegay0; omegaz0];

% Attitude
e0 = [0 deg2rad(83.9) 0]';
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

% Data extracti
N = length(out.tout);
ts = out.tout;
As_BN = out.A_BN;


%% Visualization

