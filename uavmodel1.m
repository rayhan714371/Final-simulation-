%% UAV Linearization
% This script linearizes a 6-DOF nonlinear UAV model around hover point
% Resulting model: x_dot = Ax + Bu + h(x,u) + w,  y = Cx + v
% where h(x,u) is the linearization error modeled as Gaussian noise

clear;
clc;

%% Define UAV parameters
m = 1.0;      % UAV mass (kg)
g = 9.81;     % Gravity acceleration (m/s^2)
J_x = 0.0142; % Moment of inertia around x-axis (kg*m^2)
J_y = 0.0142; % Moment of inertia around y-axis (kg*m^2)
J_z = 0.0142; % Moment of inertia around z-axis (kg*m^2)

% Save parameters for future use
params.m = m;
params.g = g;
params.J_x = J_x;
params.J_y = J_y;
params.J_z = J_z;

%% Define the state vector
% x = [p_x, p_y, p_z, v_x, v_y, v_z, phi, theta, psi, omega_x, omega_y, omega_z]
% where:
% p_x, p_y, p_z: position in world frame
% v_x, v_y, v_z: linear velocities in world frame
% phi, theta, psi: Euler angles (roll, pitch, yaw)
% omega_x, omega_y, omega_z: angular velocities in body frame

% Define the input vector
% u = [F, tau_phi, tau_theta, tau_psi]
% where:
% F: total thrust force
% tau_phi, tau_theta, tau_psi: moments around body axes

%% Equilibrium point (hover)
% At hover: phi=0, theta=0, psi arbitrary, constant altitude, zero velocity
x_eq = zeros(12, 1); % All states zero at hovering equilibrium
F_eq = m*g;          % Force to maintain hovering = mg
u_eq = [F_eq; 0; 0; 0]; % Equilibrium control input

%% Linearization
% Define system dimensions
nx = 12; % Number of states
nu = 4;  % Number of inputs
ny = 12; % Number of outputs (assuming full state feedback)

% Initialize linearized system matrices
A = zeros(nx, nx);
B = zeros(nx, nu);
C = eye(ny, nx);  % Full state feedback
D = zeros(ny, nu);

% Fill A matrix based on linearized equations
% Position derivatives are velocities
A(1,4) = 1;  % dp_x/dt = v_x
A(2,5) = 1;  % dp_y/dt = v_y
A(3,6) = 1;  % dp_z/dt = v_z

% Velocity derivatives (linearized)
A(4,8) = -g; % dv_x/dt = -g*theta
A(5,7) = g;  % dv_y/dt = g*phi
% dv_z/dt has no linearized state component

% Angle derivatives from angular velocities
A(7,10) = 1; % dphi/dt = omega_x
A(8,11) = 1; % dtheta/dt = omega_y
A(9,12) = 1; % dpsi/dt = omega_z

% B matrix - Control inputs effect
% Force affects acceleration in z direction
B(6,1) = -1/m;  % dv_z/dt = -F/m

% Moments affect angular accelerations
B(10,2) = 1/J_x; % domega_x/dt = tau_phi/J_x
B(11,3) = 1/J_y; % domega_y/dt = tau_theta/J_y
B(12,4) = 1/J_z; % domega_z/dt = tau_psi/J_z

%% Linearization error statistics (h(x,u))
% Define uncertainties for phi and theta
sigma_phi = 0.1;  % standard deviation of phi (rad)
sigma_theta = 0.1; % standard deviation of theta (rad)
sigma_psi = 0.1;   % standard deviation of psi (rad)

% Variance calculations for h(x,u) components
var_h_px = (g/2)^2 * (sigma_phi^2 * sigma_theta^2); % Var(h_p_ddot_x) = (g/2)^2 * Var(phi*theta)
var_h_py = (g/6)^2 * 15 * sigma_phi^6;              % Var(h_p_ddot_y) = (g/6)^2 * Var(phi^3)
var_h_pz = (g/2)^2 * (2*sigma_phi^4 + 2*sigma_theta^4); % Var(h_p_ddot_z)

% Q_h diagonal matrix (covariance of linearization error)
Q_h = zeros(nx);
Q_h(4,4) = var_h_px;  % p_x_ddot component
Q_h(5,5) = var_h_py;  % p_y_ddot component
Q_h(6,6) = var_h_pz;  % p_z_ddot component

% Process and measurement noise covariance matrices (examples)
Q_w = 0.01 * eye(nx);  % Process noise covariance
R_v = 0.01 * eye(ny);  % Measurement noise covariance

%% Save the linearized model and parameters
uav_model.A = A;
uav_model.B = B;
uav_model.C = C;
uav_model.D = D;
uav_model.Q_h = Q_h;
uav_model.Q_w = Q_w;
uav_model.R_v = R_v;
uav_model.x_eq = x_eq;
uav_model.u_eq = u_eq;
uav_model.params = params;
uav_model.sigma_phi = sigma_phi;
uav_model.sigma_theta = sigma_theta;
uav_model.sigma_psi = sigma_psi;

% Save to a .mat file for future use
save('uav_model_linearized.mat', 'uav_model');

%% Display results
disp('Linearized UAV Model:')
disp('A matrix:')
disp(A)
disp('B matrix:')
disp(B)
disp('C matrix:')
disp(C)
disp('Linearization error covariance (Q_h):')
disp(Q_h)

fprintf('\nModel saved to uav_model_linearized.mat\n');