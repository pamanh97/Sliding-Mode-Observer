function X_next = system_dynamics(X, U, F_ext, params, Ts)
    % Simulates the system dynamics for one time step
    
    % Extract current state
    x = X(1);
    y = X(2);
    theta = X(3);
    x_dot = X(4);
    y_dot = X(5);
    theta_dot = X(6);
    
    % Extract parameters
    m = params.m;
    Iz = params.Iz;
    r = params.r;
    lx = params.lx;
    ly = params.ly;
    Kt = params.Kt;
    Kb = params.Kb;
    R = params.R;
    tau_f = params.shaft_friction;
    
    % Local to global rotation matrix
    R_mat = [cos(theta), -sin(theta), 0;
            sin(theta), cos(theta), 0;
            0, 0, 1];
    
    % Wheel velocities from robot velocities (inverse kinematics) - Equation (2)
    wheel_vel = (1/r) * [1, -1, -(lx+ly);
                         1,  1,  (lx+ly);
                         1,  1, -(lx+ly);
                         1, -1,  (lx+ly)] * [x_dot; y_dot; theta_dot];
    
    % Motor torques from applied voltages - Equation (6)
    torques = zeros(4, 1);
    for i = 1:4
        torques(i) = Kt * (U(i) - Kb * wheel_vel(i)) / R - tau_f;
    end
    
    % Robot velocities from wheel torques (simplified dynamics)
    F_wheels = (1/r) * [1, 1, 1, 1;
                        -1, 1, 1, -1;
                        -(lx+ly), (lx+ly), -(lx+ly), (lx+ly)] * torques;
    
    % Include external forces
    F_total = F_wheels + F_ext;
    
    % Acceleration
    accel = [F_total(1)/m; F_total(2)/m; F_total(3)/Iz];
    
    % Euler integration for velocity
    vel_next = [x_dot; y_dot; theta_dot] + Ts * accel;
    
    % Euler integration for position
    pos_global = [x; y; theta] + Ts * R_mat * [x_dot; y_dot; theta_dot];
    
    % Normalize angle to [-pi, pi]
    pos_global(3) = mod(pos_global(3) + pi, 2*pi) - pi;
    
    % Next state
    X_next = [pos_global; vel_next];
end
