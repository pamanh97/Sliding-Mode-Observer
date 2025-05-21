function [alpha_MPC, alpha_PID, alpha_SMO, beta] = fuzzy_logic_controller(e_p, e_v, e_p_dot)
    % Fuzzy Logic Controller for adaptive parameter tuning
    
    % Define membership function ranges
    e_p_range = [0, 0.5];       % Position error range (m)
    e_v_range = [0, 0.3];       % Velocity error range (m/s)
    e_p_dot_range = [-0.2, 0.2]; % Rate of change of position error range (m/s)
    
    % Define output ranges
    alpha_range = [0.5, 1.5];   % Adjustment factor range
    beta_range = [0, 1];        % Control blending factor range
    
    % Normalize inputs to [0, 1] for position error and velocity error
    e_p_norm = min(max((e_p - e_p_range(1)) / (e_p_range(2) - e_p_range(1)), 0), 1);
    e_v_norm = min(max((e_v - e_v_range(1)) / (e_v_range(2) - e_v_range(1)), 0), 1);
    
    % Normalize input to [-1, 1] for rate of change of position error
    e_p_dot_norm = min(max((e_p_dot - e_p_dot_range(1)) / (e_p_dot_range(2) - e_p_dot_range(1)) * 2 - 1, -1), 1);
    
    % Compute memberships for position error
    mu_e_p_small = max(0, 1 - 2 * e_p_norm);
    mu_e_p_medium = max(0, 1 - 2 * abs(e_p_norm - 0.5));
    mu_e_p_large = max(0, 2 * e_p_norm - 1);
    
    % Compute memberships for velocity error
    mu_e_v_small = max(0, 1 - 2 * e_v_norm);
    mu_e_v_medium = max(0, 1 - 2 * abs(e_v_norm - 0.5));
    mu_e_v_large = max(0, 2 * e_v_norm - 1);
    
    % Compute memberships for rate of change of position error
    mu_e_p_dot_negative = max(0, -e_p_dot_norm);
    mu_e_p_dot_zero = max(0, 1 - 2 * abs(e_p_dot_norm));
    mu_e_p_dot_positive = max(0, e_p_dot_norm);
    
    % Fuzzy rules for MPC weight adjustment
    % Rule 1: IF e_p is SMALL AND e_v is SMALL AND e_p_dot is ZERO THEN alpha_MPC is HIGH
    rule1_mpc = min([mu_e_p_small, mu_e_v_small, mu_e_p_dot_zero]);
    
    % Rule 2: IF e_p is LARGE OR e_v is LARGE THEN alpha_MPC is MEDIUM
    rule2_mpc = max(mu_e_p_large, mu_e_v_large);
    
    % Rule 3: IF e_p_dot is LARGE (either positive or negative) THEN alpha_MPC is LOW
    rule3_mpc = max(mu_e_p_dot_negative, mu_e_p_dot_positive);
    
    % Output singleton values for alpha_MPC
    alpha_MPC_low = alpha_range(1);
    alpha_MPC_medium = (alpha_range(1) + alpha_range(2)) / 2;
    alpha_MPC_high = alpha_range(2);
    
    % Defuzzification for alpha_MPC using center of gravity
    alpha_MPC = (rule1_mpc * alpha_MPC_high + rule2_mpc * alpha_MPC_medium + rule3_mpc * alpha_MPC_low) / ...
                (rule1_mpc + rule2_mpc + rule3_mpc + eps);
    
    % Similarly for PID and SMO (simplified rules)
    % For PID: Higher gains for large errors, lower for small errors
    alpha_PID = (mu_e_p_large * alpha_range(2) + mu_e_p_small * alpha_range(1)) / ...
                (mu_e_p_large + mu_e_p_small + eps);
    
    % For SMO: Higher gains for disturbances (indicated by changing errors)
    alpha_SMO = (max(mu_e_p_dot_negative, mu_e_p_dot_positive) * alpha_range(2) + ...
                mu_e_p_dot_zero * alpha_range(1)) / ...
                (max(mu_e_p_dot_negative, mu_e_p_dot_positive) + mu_e_p_dot_zero + eps);
    
    % Control blending factor beta
    % Larger beta (more MPC) for complex maneuvers, smaller beta (more PID) for stable regions
    beta = (mu_e_p_small * 0.8 + mu_e_p_large * 0.2) / (mu_e_p_small + mu_e_p_large + eps);
    
    % Ensure outputs are within valid ranges
    alpha_MPC = min(max(alpha_MPC, alpha_range(1)), alpha_range(2));
    alpha_PID = min(max(alpha_PID, alpha_range(1)), alpha_range(2));
    alpha_SMO = min(max(alpha_SMO, alpha_range(1)), alpha_range(2));
    beta = min(max(beta, beta_range(1)), beta_range(2));
end
