%% UAV LQR Controller Design and Validation
% This script designs an LQR controller for a linearized UAV model,
% validates its performance, and simulates the closed-loop system.

clear;
clc;

%% Load Linearized UAV Model
% Assuming the linearized model is saved in 'uav_model_linearized.mat'
load('uav_model_linearized.mat'); % Contains A, B, C, D, params, etc.

% Extract system matrices
A = uav_model.A;
B = uav_model.B;
C = uav_model.C;
D = uav_model.D;
params = uav_model.params;
Q_h = uav_model.Q_h; % Linearization error covariance

%% LQR Controller Design
% Define Q and R matrices (tuning parameters)
Q = diag([10, 10, 10, 1, 1, 1, 100, 100, 100, 10, 10, 10]); % Penalize states
R = diag([10, 10, 10, 10]); % Penalize inputs

% Solve for optimal gain matrix K
[K, ~, ~] = lqr(A, B, Q, R);

% Display the gain matrix
disp('LQR Gain Matrix K:');
disp(K);

%% Validate LQR Controller - Closed-Loop Stability
% Compute closed-loop system matrix
A_cl = A - B * K;

% Check eigenvalues of the closed-loop system
eigenvalues = eig(A_cl);
disp('Closed-Loop Eigenvalues:');
disp(eigenvalues);

if all(real(eigenvalues) < 0)
    disp('Closed-loop system is stable.');
else
    disp('Closed-loop system is unstable!');
end

%% Simulate the Linearized Model
% Define simulation parameters
tspan = 0:0.01:10; % Time vector (0 to 10 seconds)
x0 = [1; 0; 0; 0; 0; 0; 0.1; 0; 0; 0; 0; 0]; % Initial disturbance

% Simulate the closed-loop linear system
[t, x_linear] = ode45(@(t, x) linearized_dynamics(t, x, A, B, K, Q_h), tspan, x0);

% Plot results
figure;
plot(t, x_linear);
xlabel('Time (s)');
ylabel('States');
title('Linear System Response with LQR Control (including linearization error)');
grid on;
legend('p_x', 'p_y', 'p_z', 'v_x', 'v_y', 'v_z', '\phi', '\theta', '\psi', '\omega_x', '\omega_y', '\omega_z');

%% Simulate the Nonlinear Model
% Simulate the nonlinear system
[t, x_nonlinear] = ode45(@(t, x) nonlinear_uav(t, x, K, params), tspan, x0);

% Plot nonlinear simulation results
figure;
plot(t, x_nonlinear);
xlabel('Time (s)');
ylabel('States');
title('Nonlinear System Response with LQR Control');
grid on;
legend('p_x', 'p_y', 'p_z', 'v_x', 'v_y', 'v_z', '\phi', '\theta', '\psi', '\omega_x', '\omega_y', '\omega_z');

%% Robustness to Noise and Uncertainty
% Define noise covariances
Q_w = uav_model.Q_w; % Process noise covariance
R_v = uav_model.R_v; % Measurement noise covariance

% Add process noise, measurement noise, and linearization error
process_noise = mvnrnd(zeros(size(A, 1), 1), Q_w, length(t))';
measurement_noise = mvnrnd(zeros(size(C, 1), 1), R_v, length(t))';
linearization_error = mvnrnd(zeros(size(A, 1), 1), Q_h, length(t))';

% Simulate with all uncertainties using manual integration
dt = tspan(2) - tspan(1);
x_full = zeros(length(t), size(A, 1));
x_full(1, :) = x0';

for i = 1:length(t)-1
    % Current state and control
    x_curr = x_full(i, :)';
    u = -K * x_curr;
    
    % Compute derivative with all uncertainties
    dx = (A - B * K) * x_curr + linearization_error(:, i) + process_noise(:, i);
    
    % Euler integration
    x_full(i+1, :) = x_curr' + dx' * dt;
end

% Add measurement noise for observed outputs
y_full = x_full + measurement_noise';

% Plot comprehensive results
figure;
plot(t, x_full);
xlabel('Time (s)');
ylabel('States');
title('Linear System Response with All Uncertainties (h(x,u), Q_w, R_v)');
grid on;
legend('p_x', 'p_y', 'p_z', 'v_x', 'v_y', 'v_z', '\phi', '\theta', '\psi', '\omega_x', '\omega_y', '\omega_z');

% Comparison between linear model, nonlinear model, and model with uncertainties
figure;
subplot(3,1,1);
plot(t, x_linear(:,7:9)); % Plot attitude states (phi, theta, psi)
title('Linear Model - Attitude');
ylabel('Angle (rad)');
grid on;
legend('\phi', '\theta', '\psi');

subplot(3,1,2);
plot(t, x_nonlinear(:,7:9)); % Plot attitude states from nonlinear model
title('Nonlinear Model - Attitude');
ylabel('Angle (rad)');
grid on;
legend('\phi', '\theta', '\psi');

subplot(3,1,3);
plot(t, x_full(:,7:9)); % Plot attitude states with all uncertainties
title('Model with All Uncertainties - Attitude');
xlabel('Time (s)');
ylabel('Angle (rad)');
grid on;
legend('\phi', '\theta', '\psi');



%% Local Functions
% Linearized system dynamics with linearization error
function dx = linearized_dynamics(t, x, A, B, K, Q_h)
    % Control law u = -Kx
    u = -K * x;
    
    % Linearized dynamics with linearization error
    % Generate a random sample from the linearization error distribution
    h_xu = mvnrnd(zeros(length(x), 1), Q_h)';
    
    % Calculate state derivative: dx/dt = (A-BK)x + h(x,u)
    dx = (A - B * K) * x + h_xu;
end

% Nonlinear UAV dynamics
function dx = nonlinear_uav(t, x, K, params)
    % States: x = [p_x; p_y; p_z; v_x; v_y; v_z; phi; theta; psi; omega_x; omega_y; omega_z]
    % Control: u = -K * x
    u = -K * x; % LQR control law
    
    % Extract states
    phi = x(7); theta = x(8); psi = x(9);
    v_x = x(4); v_y = x(5); v_z = x(6);
    omega_x = x(10); omega_y = x(11); omega_z = x(12);
    
    % Extract control inputs (with equilibrium values)
    F_eq = params.m * params.g; % Equilibrium thrust force
    F = F_eq + u(1);           % Add control input to equilibrium
    tau_phi = u(2); tau_theta = u(3); tau_psi = u(4);
    
    % Nonlinear equations
    dx = zeros(12, 1);
    dx(1:3) = [v_x; v_y; v_z]; % Position derivatives
    dx(4) = -cos(phi)*sin(theta)*F/params.m; % v_x_dot
    dx(5) = sin(phi)*F/params.m;            % v_y_dot
    dx(6) = params.g - cos(phi)*cos(theta)*F/params.m; % v_z_dot
    dx(7) = omega_x;           % phi_dot
    dx(8) = omega_y;           % theta_dot
    dx(9) = omega_z;           % psi_dot
    dx(10) = tau_phi / params.J_x;  % omega_x_dot
    dx(11) = tau_theta / params.J_y; % omega_y_dot
    dx(12) = tau_psi / params.J_z;   % omega_z_dot
end