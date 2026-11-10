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
height = 2000;
sma = R_E + height;
ecc = 0;
inc = deg2rad(0);
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
        0  0 -1]';

% Panel center points wrt geometric center
r_geo = [ 15     0   0;
         -15     0   0;
           0    10   0;
           0   -10   0;
           0     0   5;
           0     0  -5]' * 1e-2;

% Center of mass position
r_cm = [10; 0; 0] * 1e-2;

% Panel center points wrt center of mass
r_F = r_geo - r_cm;

% Panel areas
Area  = [2 2 3 3 6 6] * 1e-2;

% Specular reflection coefficients
rho_s = [0.5 0.5 0.5 0.5 0.5 0.5];

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
sT_bias = zeros(3,3); % we should define a pure stretching matrix

% Star tracker accuracy and tolerated slew rate
fi_acc = deg2rad(0.02 / 3); % divided by 3 since it's reported as 3 sigma
theta_acc = deg2rad(0.02 / 3);
psi_acc = deg2rad(0.06 / 3); % defined as roll axis

max_slew_rate = deg2rad(0.3);

%% Initial conditions

% True anomaly
ta0 = 0;

% Angular velocity
omega0 = [0.1 0.1 n]';

% Attitude
e0 = [0 0 deg2rad(20)]';

%% Simulation

% Model Parameters
usePerts = 0;

% Simulation parameters
stopTime = T;
fixedStep = 0.1;

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
omega_B = out.omega_B';
M = out.M';
M_SRP = out.M_SRP';
M_GG = out.M_GG';
M_Magn = out.M_Magn';

%% Visualization

% Angular momentum
H_B = I * omega_B;
H_mag = vecnorm(H_B, 2, 1);

H_N = zeros(3, N);
for k = 1:N
    H_N(:,k) = As_BN(:,:,k).' * H_B(:,k);
end

mag_dev = H_mag - H_mag(1);
comp_dev = H_N - H_N(:,1);

figure
tiledlayout(2,1)

nexttile
plot(ts, mag_dev)
grid on
xlabel('Time [s]')
ylabel('$\|\mathbf{H}_B\| - \|\mathbf{H}_B(0)\|$ [kg m$^2$/s]', 'Interpreter', 'latex')
title('Angular momentum magnitude deviation (body frame)', 'Interpreter', 'latex')

nexttile
plot(ts, comp_dev)
grid on
xlabel('Time [s]')
ylabel('$\mathbf{H}_N - \mathbf{H}_N(0)$ [kg m$^2$/s]', 'Interpreter', 'latex')
legend('$H_x$', '$H_y$', '$H_z$', 'Interpreter', 'latex')
title('Angular momentum components deviation (inertial frame)', 'Interpreter', 'latex')

% Perturbations
M_data = {M, M_GG, M_SRP, M_Magn};
M_titles = {'Total torque', 'Gravity gradient torque', 'SRP torque', 'Magnetic torque'};

figure
tiledlayout(2,2)

for i = 1:4
    m = M_data{i};

    nexttile
    plot(ts, m)
    grid on
    xlabel('Time [s]')
    ylabel('$\mathbf{M}$ [N m]', 'Interpreter', 'latex')
    title(M_titles{i}, 'Interpreter', 'latex')
    legend('$M_x$', '$M_y$', '$M_z$', 'Interpreter', 'latex')
end

