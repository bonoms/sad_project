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

%% Spacecraft Orbital Parameters

% Keplerian elements
height = 2000;
sma = R_E + height;
ecc = 0.001;
inc = deg2rad(30);
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
N_B = [ 1  0  0;
       -1  0  0;
        0  1  0;
        0 -1  0;
        0  0  1;
        0  0 -1;
       -1  0  0;    % Solar Panel A
       -1  0  0;    % Solar Panel B
        1  0  0;    % Solar Panel A Backside
        1  0  0]';  % Solar Panel B Backside

% Panel center points wrt geometric center
r_geo = [ 15     0   0;
         -15     0   0;
           0    10   0;
           0   -10   0;
           0     0   5;
           0     0  -5;
         -15    25   0;              % Solar Panel A
         -15   -25   0;              % Solar Panel B
         -15    25   0;              % Solar Panel A Backside
         -15   -25   0]' * 1e-2;     % Solar Panel B Backside

% Center of mass position
r_cm = [10; 3; 0] * 1e-2;

% Panel center points wrt center of mass
r_F = r_geo - r_cm;

% Panel areas
Area  = [2 2 3 3 6 6 3 3 3 3] * 1e-2;

% Specular reflection coefficients
rho_s = [0 0 0 0 0.10 0.10 0.10 0.10 0 0];

% Diffuse reflection coefficients
rho_d = [0 0 0 0 0.25 0.25 0.25 0.25 0 0];
%%
G = 1361;
etha = [0 0 0 0 0 0 0.25 0.25 0 0];
packing = 0.8;

P_vec = G*Area.*etha*packing;
P_max  = sum(P_vec);
k_scale  = 0.02;
bias_P   = 0.005*P_max;  
Ts     = 1e-3;
D_w    = 0.01*P_max*sqrt(2*Ts);   % white
D_p    = 0.005*P_max*sqrt(2*Ts);  % pink
D_b    = 1e-4*P_max*sqrt(2*Ts);   % brown
%% Sensors data

% Sensor misalignment matrix star tracker
fi_mis_sT = 0.0000001; % given in 312 notation
theta_mis_sT = 0.0000001;
psi_mis_sT = 0.0000001;

A_mis_sT = [1 fi_mis_sT -theta_mis_sT; % this should act also as any bias
            -fi_mis_sT 1 psi_mis_sT;
            fi_mis_sT -psi_mis_sT 1];

% Sensor star tracker bias matrix
sT_bias = zeros(3,3); % we should define a pure stretching matrix

% Star tracker accuracy and tolerated slew rate
fi_acc = deg2rad(0.02 / 3); % divided by 3 since it's reported as 3 sigma
theta_acc = deg2rad(0.02 / 3);
psi_acc = deg2rad(0.06 / 3); % defined as roll axis

err = [fi_acc;theta_acc;psi_acc];
max_slew_rate = deg2rad(0.3);

%% Initial conditions

% True anomaly
ta0 = 0;

% Angular velocity
omega0 = [deg2rad(0) deg2rad(0) n]';

% Attitude
e0 = [deg2rad(180) deg2rad(0) deg2rad(0)]';

%% Simulation

% Model Parameters
usePerts = 1*0;

% Simulation parameters
stopTime = T;
fixedStep = 0.1;

%% Simulink model

out = sim(fullfile(projectRoot, 'prova_alessandro_progetto_sensor.slx'), ...
    "StopTime", "stopTime", ...
    "SolverType", "Fixed-step", ...
    "Solver", "ode4", ...
    "FixedStep", "fixedStep", ...
    "ReturnWorkspaceOutputs", "on");



