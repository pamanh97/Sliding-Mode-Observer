function [Ad, Bd] = discretize_system(A, B, Ts)
    % Discretize the continuous-time system
    
    % Method 1: Zero-order hold (ZOH)
    sys_cont = ss(A, B, eye(size(A,1)), zeros(size(A,1), size(B,2)));
    sys_disc = c2d(sys_cont, Ts, 'zoh');
    
    % Extract discretized matrices
    Ad = sys_disc.A;
    Bd = sys_disc.B;
end
