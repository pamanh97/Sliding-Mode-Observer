function animate_simulation(X, X_ref, params, sim_params)
    % Create an animation of the robot's motion
    
    figure(7);
    set(gcf, 'Position', [100, 100, 800, 600]);
    
    % Determine frame rate and step size
    fps = 30;  % Frames per second
    sim_step = floor(sim_params.N / (fps * sim_params.T_final));
    sim_step = max(sim_step, 1);  % Ensure at least 1
    
    % Create animation frames
    for k = 1:sim_step:sim_params.N
        visualize_robot(X, X_ref, k, params);
        pause(0.01);  % Small pause to allow visualization
    end
end
