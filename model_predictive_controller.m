function U_mpc = model_predictive_controller(X, X_ref, U_prev, params)
    % Model Predictive Control implementation
    
    % Extract parameters
    Ad = params.Ad;
    Bd = params.Bd;
    Np = params.Np;
    Q = params.Q;
    R = params.R;
    S = params.S;
    X_min = params.X_min;
    X_max = params.X_max;
    U_min = params.U_min;
    U_max = params.U_max;
    dU_min = params.dU_min;
    dU_max = params.dU_max;
    
    % MPC formulation using quadratic programming
    % For simplicity, using a basic implementation without the full constraints
    
    % Augmented state prediction matrices
    [F, Phi] = mpc_prediction_matrices(Ad, Bd, Np);
    
    % Current state deviation
    delta_X = X - X_ref;
    
    % Cost function matrices
    H = 2 * (Phi' * kron(eye(Np), Q) * Phi + kron(eye(Np), R) + kron(diag([0; ones(Np-1, 1)]), S));
    f = 2 * Phi' * kron(eye(Np), Q) * F * delta_X;
    
    % Constraints
    % For simplicity, we only implement input constraints here
    A_ineq = [eye(4*Np); -eye(4*Np)];
    b_ineq = [repmat(U_max, Np, 1) - repmat(U_prev, Np, 1); 
              repmat(-U_min, Np, 1) + repmat(U_prev, Np, 1)];
    
    % Solve the QP problem
    try
        % Using MATLAB's quadprog function
        options = optimoptions('quadprog', 'Display', 'off');
        delta_U = quadprog(H, f, A_ineq, b_ineq, [], [], [], [], [], options);
        
        % Extract first control input
        U_mpc = U_prev + delta_U(1:4);
    catch
        % Fallback option if QP solver fails
        warning('QP solver failed. Using previous control input.');
        U_mpc = U_prev;
    end
end

function [F, Phi] = mpc_prediction_matrices(A, B, Np)
    % Create prediction matrices for MPC
    
    nx = size(A, 1);
    nu = size(B, 2);
    
    % Initialize matrices
    F = zeros(nx * Np, nx);
    Phi = zeros(nx * Np, nu * Np);
    
    % Fill matrices
    for i = 1:Np
        F((i-1)*nx+1:i*nx, :) = A^i;
        
        for j = 1:i
            Phi((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = A^(i-j) * B;
        end
    end
end
