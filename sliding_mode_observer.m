function [X_hat_next, Y_hat] = sliding_mode_observer(X_hat, U, Y, params)
    % Sliding Mode Observer implementation
    
    % Extract parameters
    A = params.A;
    B = params.B;
    C = params.C;
    L = params.L;
    K = params.K;
    delta = params.delta;
    Ts = 0.01; % Sampling time
    
    % Current estimated measurement
    Y_hat = C * X_hat;
    
    % Measurement error
    e_y = Y - Y_hat;
    
    % Switching term with smooth approximation
    switching = K * smooth_sign(e_y, delta);
    
    % SMO dynamics in continuous time
    X_hat_dot = A * X_hat + B * U + L * e_y + switching;
    
    % Euler integration for discrete time update
    X_hat_next = X_hat + Ts * X_hat_dot;
end

function s = smooth_sign(x, delta)
    % Smooth approximation of the sign function to reduce chattering
    s = x ./ (abs(x) + delta);
end
