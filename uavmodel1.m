%% 6 DOF Linearized UAV with Lumped Gaussian Uncertainty
% This script provides a linearized UAV model in the form:
% dot_x(t) = Ax(t) + Bu(t) + d(x,u) + w(t)
% y(t) = Cx(t) + v(t)
% where d(x,u) represents the lumped uncertainty (combining model uncertainty and nonlinear residuals)
% w(t) represents process noise
% and v(t) represents measurement noise

%% UAV Physical Parameters
m = 1.5;              % Mass (kg)
g = 9.81;             % Gravity (m/s^2)
L = 0.23;             % Arm length (m)

% Moments of inertia (kg.m^2)
Ixx = 0.0211;
Iyy = 0.0219;
Izz = 0.0366;

% Rotor and aerodynamic parameters
kT = 1.091e-5;        % Thrust coefficient
kD = 1.779e-7;        % Drag coefficient
kf = 0.01;            % Friction coefficient

%% State and Control Vectors Definition
% State vector x = [px; py; pz; phi; theta; psi; vx; vy; vz; p; q; r]
% where:
% (px, py, pz) - position in inertial frame
% (phi, theta, psi) - Euler angles (roll, pitch, yaw)
% (vx, vy, vz) - linear velocities in body frame
% (p, q, r) - angular velocities in body frame

% Control input vector u = [F; tau_phi; tau_theta; tau_psi]
% where:
% F - total thrust
% tau_phi - roll torque
% tau_theta - pitch torque
% tau_psi - yaw torque

%% Linearization Point (Hover)
% Define equilibrium state (hover condition)
X_eq = zeros(12, 1);
X_eq(3) = -2;  % Hover at 2 meters above ground

% Define equilibrium input (hover thrust balances gravity)
U_eq = [m*g; 0; 0; 0];

%% Linearized State-Space Matrices
% A matrix (state matrix)
A = zeros(12, 12);

% Position derivatives
A(1, 7) = 1;  % dx/dt = vx
A(2, 8) = 1;  % dy/dt = vy
A(3, 9) = 1;  % dz/dt = vz

% Attitude kinematics at hover
A(4, 10) = 1;  % dphi/dt = p
A(5, 11) = 1;  % dtheta/dt = q
A(6, 12) = 1;  % dpsi/dt = r

% Velocity dynamics (linearized around hover)
A(7, 5) = g;    % dvx/dt affected by pitch angle (gravity projection)
A(8, 4) = -g;   % dvy/dt affected by roll angle (gravity projection)
A(7, 7) = -kf/m;  % Damping in x direction
A(8, 8) = -kf/m;  % Damping in y direction
A(9, 9) = -kf/m;  % Damping in z direction

% Angular velocity dynamics
A(10, 10) = -0.01;  % Aerodynamic damping in roll
A(11, 11) = -0.01;  % Aerodynamic damping in pitch
A(12, 12) = -0.02;  % Aerodynamic damping in yaw

% B matrix (input matrix)
B = zeros(12, 4);

% Thrust affects vertical acceleration (in body frame)
B(9, 1) = -1/m;  % Negative because Z-axis points downward in body frame

% Torques affect angular accelerations
B(10, 2) = 1/Ixx;  % Roll torque
B(11, 3) = 1/Iyy;  % Pitch torque
B(12, 4) = 1/Izz;  % Yaw torque

% C matrix (output matrix)
% Assuming all states are measurable
C = eye(12);

% D matrix (direct feedthrough)
D = zeros(12, 4);

%% Noise Characteristics
% Process noise covariance matrix Q
% Process noise affects the state dynamics directly
% Generally higher for velocity and angular states, lower for position states
pos_proc_std = 0.01;      % Position process noise (m/s^2)
angle_proc_std = 0.02;    % Angle process noise (rad/s^2)
vel_proc_std = 0.05;      % Velocity process noise (m/s^2)
angvel_proc_std = 0.1;    % Angular velocity process noise (rad/s^2)

% Create diagonal process noise covariance matrix
Q = diag([0.0001 0.0001 0.0001 0.0004 0.0004 0.0004 0.0025 0.0025 0.0025 0.01 0.01 0.01]);  

% Measurement noise covariance matrix R
pos_meas_std = 0.05;      % Position measurement noise (m)
angle_meas_std = 0.02;    % Angle measurement noise (rad)
vel_meas_std = 0.1;       % Velocity measurement noise (m/s)
angvel_meas_std = 0.05;   % Angular velocity measurement noise (rad/s)

% Create diagonal measurement noise covariance matrix
R = diag([0.0025 0.0025 0.0025 0.0004 0.0004 0.0004 0.01 0.01 0.01 0.0025 0.0025 0.0025]);

% Create input process noise covariance (optional)
input_noise_std = 0.1;    % Input noise standard deviation
Qu = input_noise_std^2 * eye(4);  % Input noise covariance

%% Create Linearized State-Space Model
sys_lin = ss(A, B, C, D);

% Display linearized model properties
disp('Linearized UAV Model:');
disp('A matrix (State Matrix):');
disp(A);
disp('B matrix (Input Matrix):');
disp(B);
disp('C matrix (Output Matrix):');
disp(C);
disp('Process Noise Covariance Matrix Q:');
disp(Q);
disp('Measurement Noise Covariance Matrix R:');
disp(R);

%% Lumped Uncertainty Parameters
% Define Gaussian model for lumped uncertainty (nonlinear residuals + model uncertainty)
% The lumped uncertainty covariance is state-dependent and has different values for different state components

% Base covariance values for the lumped uncertainty
% These values represent a combination of nonlinear residuals and model uncertainty
base_uncertainty = zeros(12, 1);
base_uncertainty(1:3) = 0.01;     % Position uncertainty (m/s^2)
base_uncertainty(4:6) = 0.02;     % Angle uncertainty (rad/s^2)
base_uncertainty(7:9) = 0.05;     % Velocity uncertainty (m/s^2)
base_uncertainty(10:12) = 0.1;    % Angular velocity uncertainty (rad/s^2)

% State-dependency factors
% These factors increase the uncertainty as the state deviates from equilibrium
pos_factor = 0.02;         % Position scaling factor
angle_factor = 0.5;        % Angle scaling factor
vel_factor = 0.1;          % Velocity scaling factor
angvel_factor = 0.2;       % Angular velocity scaling factor

% Control-dependency factors
% These factors increase the uncertainty as the control deviates from equilibrium
thrust_factor = 0.05;      % Thrust scaling factor
torque_factor = 0.1;       % Torque scaling factor

%% Lumped Uncertainty Function
% This function generates the lumped uncertainty as Gaussian noise
% The covariance matrix is state and control dependent
function d_xu = lumped_uncertainty(x, u)
    % Pull parameters from base workspace
    base_uncertainty = evalin('base', 'base_uncertainty');
    X_eq = evalin('base', 'X_eq');
    U_eq = evalin('base', 'U_eq');
    pos_factor = evalin('base', 'pos_factor');
    angle_factor = evalin('base', 'angle_factor');
    vel_factor = evalin('base', 'vel_factor');
    angvel_factor = evalin('base', 'angvel_factor');
    thrust_factor = evalin('base', 'thrust_factor');
    torque_factor = evalin('base', 'torque_factor');
    
    % Calculate deviations from equilibrium
    x_dev = abs(x - X_eq);
    u_dev = abs(u - U_eq);
    
    % Calculate state-dependent scaling factors
    scaling = ones(12, 1);
    
    % Position scaling (increases with position deviation)
    scaling(1:3) = 1 + pos_factor * sum(x_dev(1:3));
    
    % Angle scaling (increases significantly with angle deviation)
    scaling(4:6) = 1 + angle_factor * sum(x_dev(4:6));
    
    % Velocity scaling
    scaling(7:9) = 1 + vel_factor * sum(x_dev(7:9));
    
    % Angular velocity scaling
    scaling(10:12) = 1 + angvel_factor * sum(x_dev(10:12));
    
    % Control input scaling
    % Thrust affects primarily vertical dynamics
    thrust_scaling = 1 + thrust_factor * u_dev(1);
    scaling(9) = scaling(9) * thrust_scaling;
    
    % Torques affect angular dynamics
    for i = 2:4
        scaling(8+i) = scaling(8+i) * (1 + torque_factor * u_dev(i));
    end
    
    % Calculate the uncertainty covariance
    sigma = base_uncertainty .* scaling;
    
    % Generate Gaussian distributed uncertainty
    d_xu = randn(12, 1) .* sigma;
    
    % Add correlation between similar states (optional)
    % This creates more realistic coupled dynamics
    correlation_matrix = eye(12);
    
    % Correlate x-y position uncertainties
    correlation_matrix(1, 2) = 0.3;
    correlation_matrix(2, 1) = 0.3;
    
    % Correlate roll-pitch uncertainties
    correlation_matrix(4, 5) = 0.4;
    correlation_matrix(5, 4) = 0.4;
    
    % Correlate linear velocities
    correlation_matrix(7, 8) = 0.5;
    correlation_matrix(8, 7) = 0.5;
    
    % Correlate angular velocities
    correlation_matrix(10, 11) = 0.6;
    correlation_matrix(11, 10) = 0.6;
    
    % Apply correlation (simplified approach)
    for i = 1:12
        for j = i+1:12
            if correlation_matrix(i, j) > 0
                avg_effect = correlation_matrix(i, j) * 0.5 * (d_xu(i) + d_xu(j));
                d_xu(i) = (1 - correlation_matrix(i, j)) * d_xu(i) + avg_effect;
                d_xu(j) = (1 - correlation_matrix(i, j)) * d_xu(j) + avg_effect;
            end
        end
    end
end

%% Function to generate measurement with noise
function y = measurement_with_noise(x)
    % Get the output matrix C from base workspace
    C = eye(12);  % Assuming all states are measured
    
    % Get noise covariance matrix from base workspace
    R = evalin('base', 'R');
    
    % Generate Gaussian noise for measurements
    v = randn(12, 1) .* sqrt(diag(R));
    
    % Calculate noisy measurements
    y = C*x + v;
end

%% Function to generate process noise
function w = process_noise()
    % Get process noise covariance from base workspace
    Q = evalin('base', 'Q');
    
    % Generate Gaussian process noise
    w = randn(12, 1) .* sqrt(diag(Q));
end

%% Sample simulation with lumped uncertainty and noise
function [t, x_true, y_noisy] = simulate_uav_with_uncertainty(tspan, x0, u)
    % Get model matrices
    A = evalin('base', 'A');
    B = evalin('base', 'B');
    
    % Time step for simulation
    dt = tspan(2) - tspan(1);
    num_steps = length(tspan);
    
    % Preallocate arrays
    x_true = zeros(12, num_steps);
    y_noisy = zeros(12, num_steps);
    x_true(:,1) = x0;
    
    % Generate noisy measurements for initial state
    y_noisy(:,1) = measurement_with_noise(x0);
    
    % Simulate system with Euler integration
    for k = 1:num_steps-1
        % Generate process noise for this time step
        w = process_noise();
        
        % Generate lumped uncertainty (model uncertainty + nonlinear residuals)
        d = lumped_uncertainty(x_true(:,k), u);
        
        % Update state with process noise and lumped uncertainty
        x_true(:,k+1) = x_true(:,k) + dt * (A*x_true(:,k) + B*u + d + w);
        
        % Generate noisy measurement
        y_noisy(:,k+1) = measurement_with_noise(x_true(:,k+1));
    end
    
    t = tspan;
end

%% Robust Kalman Filter Implementation considering lumped uncertainty
function [x_est, P] = robust_kalman_filter(y_noisy, u, tspan)
    % Get model matrices
    A = evalin('base', 'A');
    B = evalin('base', 'B');
    C = evalin('base', 'C');
    Q = evalin('base', 'Q');
    R = evalin('base', 'R');
    
    % Base uncertainty parameters
    base_uncertainty = evalin('base', 'base_uncertainty');
    
    % Time step for simulation
    dt = tspan(2) - tspan(1);
    num_steps = length(tspan);
    
    % Initial state estimate and covariance
    x_est = zeros(12, num_steps);
    x_est(:,1) = zeros(12, 1);  % Initial state estimate (zero or user-defined)
    P = cell(num_steps, 1);
    P{1} = eye(12);  % Initial covariance estimate (identity or user-defined)
    
    % Discrete-time system matrices (Euler approximation)
    Ad = eye(12) + dt*A;
    Bd = dt*B;
    
    % Process noise covariance (discretized)
    Qd = dt*Q;
    
    % Kalman filter iterations
    for k = 1:num_steps-1
        % Estimate the lumped uncertainty covariance based on current state estimate
        % This is a simplified approach where we use the base uncertainty values
        % scaled by a factor representing the deviation from equilibrium
        x_dev = abs(x_est(:,k) - evalin('base', 'X_eq'));
        scaling = 1 + 0.1 * sum(x_dev);
        
        % Enhanced process noise covariance accounting for lumped uncertainty
        Q_enhanced = Qd + dt^2 * diag((scaling * base_uncertainty).^2);
        
        % Predict step with augmented process noise
        x_pred = Ad*x_est(:,k) + Bd*u;
        P_pred = Ad*P{k}*Ad' + Q_enhanced;
        
        % Update step (measurement)
        K = P_pred*C'/(C*P_pred*C' + R);  % Kalman gain
        x_est(:,k+1) = x_pred + K*(y_noisy(:,k+1) - C*x_pred);
        P{k+1} = (eye(12) - K*C)*P_pred;
    end
end

%% Example usage:
% % Define simulation parameters
% tspan = 0:0.01:10;  % 10 seconds with 0.01s time step
% x0 = zeros(12, 1);  % Initial state at origin
% u = [m*g + 0.1; 0.01; 0.01; 0];  % Control input (slightly off from hover)
% 
% % Run simulation with lumped uncertainty
% [t, x_true, y_noisy] = simulate_uav_with_uncertainty(tspan, x0, u);
% 
% % Run robust Kalman filter accounting for lumped uncertainty
% [x_est, P] = robust_kalman_filter(y_noisy, u, tspan);
% 
% % Plot results
% figure;
% subplot(3,1,1);
% plot(t, x_true(1,:), 'b-', t, x_est(1,:), 'r--');
% legend('True x position', 'Estimated x position');
% title('UAV Position Tracking with Lumped Uncertainty');
% 
% subplot(3,1,2);
% plot(t, x_true(4,:), 'b-', t, x_est(4,:), 'r--');
% legend('True roll angle', 'Estimated roll angle');
% title('UAV Attitude Tracking');
% 
% subplot(3,1,3);
% plot(t, x_true(7,:), 'b-', t, x_est(7,:), 'r--');
% legend('True velocity (x)', 'Estimated velocity (x)');
% title('UAV Velocity Tracking');