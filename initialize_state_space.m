function [A, B, C] = initialize_state_space(params)
    % Initialize state-space matrices for the Mecanum robot
    
    % State vector: X = [x, y, θ, x_dot, y_dot, θ_dot]'
    % Input vector: U = [V1, V2, V3, V4]' (motor voltages)
    
    % Inertia matrix
    M = diag([params.m, params.m, params.Iz]);
    
    % Viscous friction matrix (simplified)
    Fv = 0.1 * eye(3);
    
    % Motor constant matrix (simplified mapping from voltage to torque)
    Km = params.Kt / params.R * [1, 1, 1, 1;
                               -1, 1, 1, -1;
                               -1/(params.lx + params.ly), 1/(params.lx + params.ly), -1/(params.lx + params.ly), 1/(params.lx + params.ly)];
    Km = params.r/4 * Km;
    
    % Input transformation matrix B (simplified)
    B_input = eye(4);
    
    % State matrix A
    A = [zeros(3,3), eye(3);
        zeros(3,3), -inv(M)*Fv];
    
    % Input matrix B
    B = [zeros(3,4);
        inv(M)*Km];
    
    % Output matrix C (position measurements)
    C = [eye(3), zeros(3,3)];
end
