%% Inverted Pendulum Parameters
% Run this script before opening Simulink model

clc; clear;

%% Physical Parameters
M  = 0.5;     % Cart mass (kg)
m  = 0.2;     % Pendulum mass (kg)
l  = 0.3;     % Pendulum length (m) (to center of mass)
G  = 9.81;    % Gravity (m/s^2)

%% Friction / Damping
b  = 0.1;     % Cart damping coefficient (N/m/s)
C  = 0.01;    % Pendulum rotational damping (N*m*s)

%% Inertia
J  = (1/3)*m*l^2;   % Pendulum moment of inertia (rod about pivot)

%% Initial Conditions
x0        = 0;          % Cart position (m)
xdot0     = 0;          % Cart velocity (m/s)
theta0    = deg2rad(5); % Initial angle (rad) (small disturbance)
thetadot0 = 0;          % Angular velocity (rad/s)

%% Simulation Settings
t_end = 10;    % Simulation time (s)

%% Input Force
F = 0;   % External force (N) (you can later replace with controller)

%% Derived Terms (optional but nice for clarity)
Mt = M + m;   % Total mass