% UAV LQR Controller Design
% Date: 2025-04-10
% Author: rayhan714371

% Load linearized model parameters
% If running separately, uncomment and run linearize_uav_model.m first

% Parameters
g = 9.81;    % Gravitational acceleration [m/s^2]
m = 1.0;     % UAV mass [kg]
Jx = 0.01;   % Moment of inertia around x-axis [kg.m^2]
Jy = 0.01;   % Moment of inertia around y-axis [kg.m^2]
Jz = 0.02;   % Moment of inertia around z-axis [kg.m^2]

% Define state indices for clarity
IX_PX = 1;      % Position x
IX_PY = 2;      % Position y
IX_PZ = 3;      % Position z
IX_VX = 4;      % Velocity x
IX_VY = 5;      % Velocity y
IX_VZ = 6;      % Velocity z
IX_PHI = 7;     % Roll angle
IX_THETA = 8;   % Pitch angle
IX_PSI = 9;     % Yaw angle
IX_PHI_DOT = 10;   % Roll rate
IX_THETA_DOT = 11; % Pitch rate
IX_PSI_DOT = 12;   % Yaw rate

% Define input indices
IX_F = 1;       % Total thrust
IX_TAU_PHI = 2; % Roll torque
IX_TAU_THETA = 3; % Pitch torque
IX_TAU_PSI = 4; % Yaw torque

% Hover state (equilibrium point)
x_eq = zeros(12, 1);
% In hover, the only non-zero input is the thrust to counteract gravity
F_eq = m * g;
tau_phi_eq = 0;
tau_theta_eq = 0;
tau_psi_eq = 0;
u_eq = [F_eq; tau_phi_eq; tau_theta_eq; tau_psi_eq];

% State space matrices initialization
A = zeros(12, 12);
B = zeros(12, 4);

% Fill A matrix based on linearization
% Position derivatives = velocity
A(IX_PX, IX_VX) = 1;
A(IX_PY, IX_VY) = 1;
A(IX_PZ, IX_VZ) = 1;

% Velocity derivatives from linearization
% p(ddot)_x = -g*theta
A(IX_VX, IX_THETA) = -g;

% p(ddot)_y = g*phi
A(IX_VY, IX_PHI) = g;

% p(ddot)_z = -ΔF/m (where ΔF = F - F_eq)
% This will be represented in B matrix

% Angle derivatives = angular rates
A(IX_PHI, IX_PHI_DOT) = 1;
A(IX_THETA, IX_THETA_DOT) = 1;
A(IX_PSI, IX_PSI_DOT) = 1;

% Angular acceleration terms
% phi(ddot) = tau_phi/J_x
% theta(ddot) = tau_theta/J_y
% psi(ddot) = tau_psi/J_z
% These will be represented in B matrix

% Fill B matrix based on linearization
% Force impact on acceleration
B(IX_VZ, IX_F) = -1/m;  % z-acceleration depends on thrust variation

% Torques impact on angular accelerations
B(IX_PHI_DOT, IX_TAU_PHI) = 1/Jx;
B(IX_THETA_DOT, IX_TAU_THETA) = 1/Jy;
B(IX_PSI_DOT, IX_TAU_PSI) = 1/Jz;

% Define C matrix for output (assuming we can measure all states)
C = eye(12);

% Define D matrix (direct feedthrough)
D = zeros(12, 4);

% Create state-space model
uav_ss = ss(A, B, C, D);

% Check controllability
Co = ctrb(A, B);
disp('Rank of controllability matrix:');
rank_Co = rank(Co);
disp(rank_Co);
if rank_Co == size(A, 1)
    disp('System is controllable');
else
    disp('System is not fully controllable');
    % Identify the uncontrollable modes
    [T, A_can] = ctrbf(A, B);
    % Display uncontrollable modes
    eig_uncontrollable = eig(A_can(rank_Co+1:end, rank_Co+1:end));
    disp('Uncontrollable eigenvalues:');
    disp(eig_uncontrollable);
end

% Check observability
Ob = obsv(A, C);
disp('Rank of observability matrix:');
rank_Ob = rank(Ob);
disp(rank_Ob);
if rank_Ob == size(A, 1)
    disp('System is observable');
else
    disp('System is not fully observable');
    % Identify the unobservable modes
    [T, A_can] = obsvf(A, C);
    % Display unobservable modes
    eig_unobservable = eig(A_can(rank_Ob+1:end, rank_Ob+1:end));
    disp('Unobservable eigenvalues:');
    disp(eig_unobservable);
end

% Design LQR controller
% State cost matrix
Q = diag([10 10 10 5 5 5 10 10 1 1 1 0.1]);    
% Control cost matrix
R = diag([1 0.1 0.1 0.1]);            

% Compute LQR gain matrix
[K, S, e] = lqr(A, B, Q, R);

% Display LQR gain matrix
disp('LQR Gain Matrix:');
disp(K);

% Closed loop system
A_cl = A - B*K;
sys_cl = ss(A_cl, B, C, D);

% System eigenvalues
disp('Open-loop eigenvalues:');
disp(eig(A));
disp('Closed-loop eigenvalues:');
disp(eig(A_cl));

% Simulate the closed-loop response
t = 0:0.01:5;  % Simulation time (5 seconds)

% Initial state deviation from hover
x0 = zeros(12, 1);
x0(IX_PX) = 1;     % 1m error in x position
x0(IX_PZ) = -0.5;  % -0.5m error in z position
x0(IX_PSI) = 0.2;  % 0.2 rad error in yaw angle

% Step response
[y, t, x] = initial(sys_cl, x0, t);

% Plot the position response
figure(1);
subplot(3, 1, 1);
plot(t, y(:, IX_PX));
title('X Position');
xlabel('Time (s)');
ylabel('Position (m)');
grid on;

subplot(3, 1, 2);
plot(t, y(:, IX_PY));
title('Y Position');
xlabel('Time (s)');
ylabel('Position (m)');
grid on;

subplot(3, 1, 3);
plot(t, y(:, IX_PZ));
title('Z Position');
xlabel('Time (s)');
ylabel('Position (m)');
grid on;

% Plot the attitude response
figure(2);
subplot(3, 1, 1);
plot(t, y(:, IX_PHI) * 180/pi);
title('Roll Angle');
xlabel('Time (s)');
ylabel('Angle (deg)');
grid on;

subplot(3, 1, 2);
plot(t, y(:, IX_THETA) * 180/pi);
title('Pitch Angle');
xlabel('Time (s)');
ylabel('Angle (deg)');
grid on;

subplot(3, 1, 3);
plot(t, y(:, IX_PSI) * 180/pi);
title('Yaw Angle');
xlabel('Time (s)');
ylabel('Angle (deg)');
grid on;

% Calculate control inputs during the simulation
u = zeros(4, length(t));
for i = 1:length(t)
    % Control law: u = u_eq - K * (x - x_eq)
    % Since x_eq is zero, u = u_eq - K * x
    u(:, i) = u_eq - K * x(i, :)';
end

% Plot control inputs
figure(3);
subplot(4, 1, 1);
plot(t, u(IX_F, :));
title('Thrust');
xlabel('Time (s)');
ylabel('Force (N)');
grid on;

subplot(4, 1, 2);
plot(t, u(IX_TAU_PHI, :));
title('Roll Torque');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;

subplot(4, 1, 3);
plot(t, u(IX_TAU_THETA, :));
title('Pitch Torque');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;

subplot(4, 1, 4);
plot(t, u(IX_TAU_PSI, :));
title('Yaw Torque');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;

% Function to simulate the closed-loop system with the nonlinear model
function simulate_nonlinear_vs_linear()
    % This function would implement a nonlinear simulation
    % with the LQR controller for comparison
    % Not fully implemented in this code
end