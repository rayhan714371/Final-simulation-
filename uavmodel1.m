% UAV Linearization without symbolic toolbox
% State vector: [px, py, pz, vx, vy, vz, phi, theta, psi, phi_dot, theta_dot, psi_dot]

% Parameters
g = 9.81;    % Gravitational acceleration [m/s^2]
m = 1.0;     % UAV mass [kg]
Jx = 0.01;   % Moment of inertia around x-axis [kg.m^2]
Jy = 0.01;   % Moment of inertia around y-axis [kg.m^2]
Jz = 0.02;   % Moment of inertia around z-axis [kg.m^2]

% Hover state (equilibrium point)
x_eq = zeros(12, 1);
% In hover, the only non-zero input is the thrust to counteract gravity
F_eq = m * g;
tau_phi_eq = 0;
tau_theta_eq = 0;
tau_psi_eq = 0;
u_eq = [F_eq; tau_phi_eq; tau_theta_eq; tau_psi_eq];

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

% Define uncertainty covariance based on provided information
% h(x,u) = [-g/2(ϕθ), g/6(ϕ^3), g/2(ϕ^2+θ^2), 0, 0, 0, 0, 0, 0, 0, 0, 0]'

% Variance parameters for states
sigma_phi = 0.1;    % Standard deviation of phi [rad]
sigma_theta = 0.1;  % Standard deviation of theta [rad]
sigma_psi = 0.1;    % Standard deviation of psi [rad]

% Calculate variance for each uncertainty component
% For h_p(ddot)_x = -g/2(ϕθ)
var_h_px = (g/2)^2 * (sigma_phi^2 * sigma_theta^2);

% For h_p(ddot)_y = g/6(ϕ^3)
var_h_py = (g/6)^2 * 15 * sigma_phi^6;

% For h_p(ddot)_z = g/2(ϕ^2+θ^2)
var_h_pz = (g/2)^2 * (2*sigma_phi^4 + 2*sigma_theta^4);

% Define Q_h (covariance matrix for the lumped uncertainty)
Q_h = zeros(12);
Q_h(IX_VX, IX_VX) = var_h_px;
Q_h(IX_VY, IX_VY) = var_h_py;
Q_h(IX_VZ, IX_VZ) = var_h_pz;

% Display the resulting linearized model
disp('Linearized UAV Model:');
disp('A matrix:');
disp(A);
disp('B matrix:');
disp(B);
disp('C matrix:');
disp(C);
disp('Linearization uncertainty covariance Q_h:');
disp(Q_h);

% Define a function that calculates the nonlinear dynamics
function x_dot = uav_dynamics_nonlinear(x, u)
    % Parameters
    g = 9.81;    % Gravitational acceleration [m/s^2]
    m = 1.0;     % UAV mass [kg]
    Jx = 0.01;   % Moment of inertia around x-axis [kg.m^2]
    Jy = 0.01;   % Moment of inertia around y-axis [kg.m^2]
    Jz = 0.02;   % Moment of inertia around z-axis [kg.m^2]
    
    % Extract states
    phi = x(7);      % Roll angle
    theta = x(8);    % Pitch angle
    psi = x(9);      % Yaw angle
    vx = x(4);       % Velocity x
    vy = x(5);       % Velocity y
    vz = x(6);       % Velocity z
    phi_dot = x(10);    % Roll rate
    theta_dot = x(11);  % Pitch rate
    psi_dot = x(12);    % Yaw rate
    
    % Extract inputs
    F = u(1);         % Total thrust
    tau_phi = u(2);   % Roll torque
    tau_theta = u(3); % Pitch torque
    tau_psi = u(4);   % Yaw torque
    
    % Initialize derivative vector
    x_dot = zeros(12, 1);
    
    % Position derivatives
    x_dot(1) = vx;
    x_dot(2) = vy;
    x_dot(3) = vz;
    
    % Velocity derivatives (nonlinear model)
    x_dot(4) = -cos(phi) * sin(theta) * F/m;
    x_dot(5) = sin(phi) * F/m;
    x_dot(6) = g - cos(phi) * cos(theta) * F/m;
    
    % Angle derivatives
    x_dot(7) = phi_dot;
    x_dot(8) = theta_dot;
    x_dot(9) = psi_dot;
    
    % Angular acceleration derivatives
    x_dot(10) = tau_phi/Jx;
    x_dot(11) = tau_theta/Jy;
    x_dot(12) = tau_psi/Jz;
end

% To validate the linearization, we can implement a simple simulation
% that compares the nonlinear and linearized models
function validate_linearization()
    % Parameters
    dt = 0.01;               % Time step [s]
    t_end = 5;               % Simulation duration [s]
    t = 0:dt:t_end;          % Time vector
    n_steps = length(t);     % Number of simulation steps
    
    % Initial states (small perturbations from hover)
    x_nonlin = zeros(12, n_steps);
    x_linear = zeros(12, n_steps);
    
    % Add small initial perturbation
    x_nonlin(:,1) = [0; 0; 0; 0.1; 0; 0; 0.05; 0.05; 0; 0; 0; 0];
    x_linear(:,1) = x_nonlin(:,1);
    
    % Equilibrium input (just enough thrust to hover)
    u_eq = [9.81; 0; 0; 0];  % [F, tau_phi, tau_theta, tau_psi]
    
    % Get linearized model matrices
    % ... (these would be calculated as above)
    
    % Simulate both models
    for i = 1:(n_steps-1)
        % Nonlinear simulation
        x_dot_nonlin = uav_dynamics_nonlinear(x_nonlin(:,i), u_eq);
        x_nonlin(:,i+1) = x_nonlin(:,i) + x_dot_nonlin * dt;
        
        % Linear simulation
        x_dot_linear = A * (x_linear(:,i) - x_eq) + B * (u_eq - u_eq);
        x_linear(:,i+1) = x_linear(:,i) + x_dot_linear * dt;
    end
    
    % One could plot the results to compare the models
    % This would be done in a separate function or script
end