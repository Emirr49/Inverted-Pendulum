%% Inverted Pendulum Parameters

clc; clear;

%% Physical Parameters for Inverted Pendulum (Cart-Pole System)

M_c = 1.5;     % Mass of the cart (kg)
m   = 0.2;     % Mass of the pendulum bob (kg)
M_t = M_c + m; % Total mass (kg)
l   = 0.3;     % Length to pendulum COM (m)
J   = (1/3)*m*l^2 ;   % Pendulum moment of inertia (kg*m^2) % 0.006
b   = 0.1;     % Cart friction coefficient (N*s/m)
C   = 0.001;   % Pole rotational damping (N*m*s/rad)
g   = 9.81;    % Gravitational acceleration (m/s^2)

%% Initial Conditions

x0         = 0;          % Cart position (m)
x_dot0     = 0;          % Cart velocity (m/s)
theta0     = pi - 0.05;  % Start ~3 degrees off-center so it kicks instantly at t=0 ; % Initial angle (rad) % for (small disturbance) use "deg2rad(5)" % use "pi" for swing up
theta_dot0 = 0;          % Angular velocity (rad/s)

%% Simulation Settings
t_end      = 10;         % Simulation time (s)

%% Phase 5: Linearization & LQR Balancing Design
D0 = M_t*(J + m*l^2) - (m*l)^2;

% State Space Matrices around [x; x_dot; theta; theta_dot] = [0; 0; 0; 0]
A = [0, 1, 0, 0;
     0, -b*(J + m*l^2)/D0,  (m^2*g*l^2)/D0,  -C*m*l/D0;
     0, 0, 0, 1;
     0, -b*m*l/D0,          M_t*m*g*l/D0,    -C*M_t/D0];

B = [0; (J + m*l^2)/D0; 0; m*l/D0];

% LQR Weighting Matrices
% Prioritize keeping the cart near center (Q11) and the pole vertical (Q33)
Q = diag([10, 1, 100, 1]); 
R = 0.01; % Control effort penalty

% Compute optimal gain vector K
K = lqr(A, B, Q, R);

alpha = deg2rad(15); % Angle threshold: 0.2618 rad (Must be close to vertical upright 0)
beta  = 1.5;         % Angular velocity threshold: 1.5000 rad/s (Must be slow enough to catch)

disp('Inverted Pendulum parameters loaded successfully.');