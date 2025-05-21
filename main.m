%% Main Script for Sliding Mode Observer-Based MPC with Fuzzy-PID Integration
% Implementation based on the research paper by Duc-Anh Pham and Seung-Hun Han
% Gyeongsang National University, Republic of Korea

clear all;
close all;
clc;

%% Robot Parameters
params = initialize_robot_params();

%% Simulation Parameters
sim_params.Ts = 0.01;           % Sampling time (s)
sim_params.T_final = 40;        % Total simulation time (s)
sim_params.t = 0:sim_params.Ts:sim_params.T_final; % Time vector
sim_params.N = length(sim_params.t);  % Number of simulation steps

%% Controller Parameters
% SMO parameters
controller_params.L = [15*eye(3); 60*eye(3)];  % Observer gain matrix
controller_params.K = [5*eye(3); 20*eye(3)];   % Switching gain matrix
controller_params.delta = 0.01;                % Smoothing parameter

% MPC parameters
controller_params.Np = 10;       % Prediction horizon
controller_params.Q = diag([10 10 5 1 1 0.5]); % State error weight matrix
controller_params.R = 0.1*eye(4);              % Control input weight matrix
controller_params.S = 0.5*eye(4);              % Control input change weight matrix

% State and input constraints
controller_params.X_min = [-inf -inf -inf -1.2 -1.2 -2]';
controller_params.X_max = [inf inf inf 1.2 1.2 2]';
controller_params.U_min = [-12 -12 -12 -12]';  % Voltage constraints (V)
controller_params.U_max = [12 12 12 12]';      % Voltage constraints (V)
controller_params.dU_min = [-5 -5 -5 -5]';     % Voltage rate constraints (V)
controller_params.dU_max = [5 5 5 5]';         % Voltage rate constraints (V)

% Kalman-PID parameters
controller_params.Kp = 0.5;      % Proportional gain
controller_params.Ki = 0.3;      % Integral gain
controller_params.Kd = 1.5;      % Derivative gain
controller_params.Rn = 0.01*eye(3); % Measurement noise covariance
controller_params.Qn = diag([0.001 0.001 0.001 0.01 0.01 0.01]); % Process noise covariance

% Initialize controllers
[A, B, C] = initialize_state_space(params);
controller_params.A = A;
controller_params.B = B;
controller_params.C = C;

% Discretize the system
[Ad, Bd] = discretize_system(A, B, sim_params.Ts);
controller_params.Ad = Ad;
controller_params.Bd = Bd;

%% Select Trajectory Type
% 1: Circular, 2: Square, 3: Figure-Eight, 4: S-curve
trajectory_type = 1;

%% Initialize State Variables and Measurement Noise
X = zeros(6, sim_params.N);         % System states [x, y, θ, x_dot, y_dot, θ_dot]
X_hat = zeros(6, sim_params.N);     % Estimated states
X_hat(:,1) = [0; 0; 0; 0; 0; 0];    % Initial state estimate
P = zeros(6, 6, sim_params.N);      % Error covariance matrix for Kalman filter
P(:,:,1) = eye(6);                  % Initial error covariance

Y = zeros(3, sim_params.N);         % Measurement [x, y, θ]
Y_noise = zeros(3, sim_params.N);   % Noisy measurements
Y_hat = zeros(3, sim_params.N);     % Estimated measurements

U_mpc = zeros(4, sim_params.N);     % MPC control inputs
U_pid = zeros(4, sim_params.N);     % PID control inputs
U = zeros(4, sim_params.N);         % Final hybrid control inputs

e_p = zeros(1, sim_params.N);       % Position error magnitude
e_v = zeros(1, sim_params.N);       % Velocity error magnitude
e_p_dot = zeros(1, sim_params.N);   % Rate of change of position error

alpha_MPC = ones(1, sim_params.N);  % MPC weight adjustment factor
alpha_PID = ones(1, sim_params.N);  % PID gain adjustment factor
alpha_SMO = ones(1, sim_params.N);  % SMO gain adjustment factor
beta = zeros(1, sim_params.N);      % Control blending factor
beta(1) = 0.7;                      % Initial control blending factor

% Initialize integral error for PID
e_int = zeros(3, 1);
e_prev = zeros(3, 1);

%% Generate Reference Trajectory
[X_ref, V_ref] = generate_reference_trajectory(trajectory_type, sim_params.t);

%% Main Simulation Loop
for k = 1:sim_params.N-1
    % Current time
    t_current = sim_params.t(k);
    
    % Reference trajectory at current time
    x_ref = X_ref(:, k);
    v_ref = V_ref(:, k);
    
    % Add measurement noise
    noise_pos = 0.01 * randn(2, 1);  % Position noise (m)
    noise_theta = 0.01 * randn;      % Orientation noise (rad)
    Y_noise(:, k) = [X(1:3, k) + [noise_pos; noise_theta]];
    
    % Sliding Mode Observer
    [X_hat(:, k+1), Y_hat(:, k)] = sliding_mode_observer(X_hat(:, k), U(:, k), Y_noise(:, k), controller_params);
    
    % Kalman Filter-Based PID Controller
    [X_hat_kalman, P(:,:,k+1), U_pid(:, k)] = kalman_pid_controller(X_hat(:, k), P(:,:,k), Y_noise(:, k), v_ref, e_int, e_prev, controller_params);
    
    % Update integral and previous error for PID
    e_current = v_ref - X_hat(4:6, k);
    e_int = e_int + e_current * sim_params.Ts;
    e_prev = e_current;
    
    % Model Predictive Controller
    U_mpc(:, k) = model_predictive_controller(X_hat(:, k), x_ref, U(:, max(1,k-1)), controller_params);
    
    % Calculate error metrics for fuzzy controller
    if k > 1
        e_p(k) = norm(X(1:3, k) - x_ref(1:3));
        e_v(k) = norm(X(4:6, k) - x_ref(4:6));
        e_p_dot(k) = (e_p(k) - e_p(k-1)) / sim_params.Ts;
    end
    
    % Fuzzy Logic Controller for adaptive parameter tuning
    [alpha_MPC(k), alpha_PID(k), alpha_SMO(k), beta(k)] = fuzzy_logic_controller(e_p(k), e_v(k), e_p_dot(k));
    
    % Adjust controller parameters based on fuzzy output
    controller_params.L = controller_params.L * alpha_SMO(k);
    controller_params.K = controller_params.K * alpha_SMO(k);
    controller_params.Q = controller_params.Q * alpha_MPC(k);
    controller_params.Kp = controller_params.Kp * alpha_PID(k);
    controller_params.Ki = controller_params.Ki * alpha_PID(k);
    controller_params.Kd = controller_params.Kd * alpha_PID(k);
    
    % Integrate control inputs (weighted combination of MPC and PID)
    U(:, k) = beta(k) * U_mpc(:, k) + (1 - beta(k)) * U_pid(:, k);
    
    % Apply input constraints
    U(:, k) = min(max(U(:, k), controller_params.U_min), controller_params.U_max);
    
    % Apply external disturbances if specified time periods
    F_ext = zeros(3, 1);
    if t_current >= 10 && t_current < 12
        F_ext(1) = 5; % X-direction force disturbance (N)
    end
    if t_current >= 20 && t_current < 22
        F_ext(2) = 5; % Y-direction force disturbance (N)
    end
    
    % System dynamics simulation
    X(:, k+1) = system_dynamics(X(:, k), U(:, k), F_ext, params, sim_params.Ts);
    
    % Generate measurements
    Y(:, k+1) = X(1:3, k+1);
end

%% Calculate Performance Metrics
metrics = calculate_performance_metrics(X, X_ref, U, sim_params);

%% Plot Results
plot_results(X, X_ref, U, U_mpc, U_pid, beta, e_p, sim_params);
