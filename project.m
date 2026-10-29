% load libraries and other files

projectRoot = fileparts(mfilename('fullpath'));
cd(projectRoot);
addpath(genpath(projectRoot));

addpath('/Users/bonoms/Desktop/uni/own_files/sad/matlab/common')
load_system('laythe');

%% constants

mu_E = 398600.4418;
R_E  = 6371;
w_E = 7.292115e-5;

T_sun = 365*24*60*60;

r_sun = 149597870.7; % distance in km
epsilon = deg2rad(23.45);

e = 0.002;

i_orbit = deg2rad(97.32);

theta_0 = 0;

h = 449;
a = R_E + h;

n = sqrt(mu_E/a^3);
T = 2*pi/n;


% constants
      
Ix = 0.087;
Iy = 0.067;
Iz = 0.033; 

j_b = [0.01 0.05 0.01]; %current flowing

I = diag([Ix Iy Iz]);
Inv_I = inv(I);

wx_0 = deg2rad(1);
wy_0 = deg2rad(1);
wz_0 = deg2rad(1);
w_0 = [wx_0, wy_0, wz_0]*0;

e0 = [0 0 0]';
A0 = eye(3,3); % not necessary but to make dcm block work 

%% srp

N_B = [ 1 -1  0  0  0  0  ;
        0  0  1 -1  0  0  ;
        0  0  0  0  1 -1  ];

r_F = [5 -25  0   0   0   0 ;
       0   0  10 -10  0   0 ;
       0   0   0   0   5  -5 ] * 1e-2;

rho_s = [0.15 0.5 0.5 0.5 0.5 0.5]; % to change ..
rho_d = [0.05 0.05 0.05 0.05 0.05 0.05]; %% to change based on actual spacecraft
Area  = [2 2 0 3 6 6] * 1e-2;

%% sim settings

out = sim("aura_mission.slx", ...
    "StopTime", "10*T", ...
    "SolverType", "Fixed-step", ...
    "Solver", "ode4", ...
    "FixedStep", "0.1", ...
    "ReturnWorkspaceOutputs", "on");


%% plots

% plot 1

N = length(out.tout);
As = out.Ab_n;

fig = figure('Color', 'w', 'Name', 'Attitude Kinematics Animation');
hold on; grid on; axis equal;
xlim([-1.5, 1.5]); ylim([-1.5, 1.5]); zlim([-1.5, 1.5]);
view(3);
hX = quiver3(0,0,0, 1,0,0, 0, 'r', 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
hY = quiver3(0,0,0, 0,1,0, 0, 'g', 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
hZ = quiver3(0,0,0, 0,0,1, 0, 'b', 'LineWidth', 2.5, 'MaxHeadSize', 0.5);
for k = 1:100:N
    A = As(:, :, k);
    X_body = A(1, :); 
    Y_body = A(2, :);
    Z_body = A(3, :);
    set(hX, 'UData', X_body(1), 'VData', X_body(2), 'WData', X_body(3));
    set(hY, 'UData', Y_body(1), 'VData', Y_body(2), 'WData', Y_body(3));
    set(hZ, 'UData', Z_body(1), 'VData', Z_body(2), 'WData', Z_body(3));
    drawnow;
    pause(0.0001);
end

