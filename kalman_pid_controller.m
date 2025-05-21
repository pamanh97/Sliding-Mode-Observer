function [X_hat, P_next, U_pid] = kalman_pid_controller(X_hat, P, Y, V_ref, e_int, e_prev, params)
    % Kalman Filter-based PID controller
    
    % Extract parameters
    Ad = params.Ad;
    Bd = params.Bd;
    C = params.C;
    Rn = params.Rn;
    Qn = params.Qn;
    Kp = params.Kp;
    Ki = params.Ki;
    Kd = params.Kd;
    Ts = 0.01; % Sampling time
    
    % Prediction step
    X_hat_pred = Ad * X_hat;
    P_pred = Ad * P * Ad' + Qn;
    
    % Update step
    Y_pred = C * X_hat_pred;
    S = C * P_pred * C' + Rn;
    K = P_pred * C' / S;
    X_hat = X_hat_pred + K * (Y - Y_pred);
    P_next = (eye(6) - K * C) * P_pred;
    
    % Extract estimated velocities
    V_hat = X_hat(4:6);
    
    % Calculate velocity error
    e_v = V_ref - V_hat;
    
    % PID control
    u_p = Kp * e_v;
    u_i = Ki * e_int;
    u_d = Kd * (e_v - e_prev) / Ts;
    
    % PID control signal (for velocity)
    u_pid = u_p + u_i + u_d;
    
    % Convert velocity control to wheel voltages
    % Using the inverse kinematics from Eq. (2) in the paper
    r = 0.04; % Wheel radius
    lx = 0.1228; % Half robot width
    ly = 0.15; % Half robot length
    
    % Wheel angular velocities
    w = (1/r) * [1, -1, -(lx+ly);
                 1,  1,  (lx+ly);
                 1,  1, -(lx+ly);
                 1, -1,  (lx+ly)] * u_pid;
    
    % Convert to motor voltages using motor model from Eq. (6)
    Kt = 2.12; % Torque constant
    Kb = 0.176; % Back-EMF constant
    R = 3; % Motor resistance
    tau_f = 0.01; % Friction torque
    
    % Motor voltages
    U_pid = zeros(4, 1);
    for i = 1:4
        U_pid(i) = (R / Kt) * (tau_f + w(i) * Kb);
    end
end
