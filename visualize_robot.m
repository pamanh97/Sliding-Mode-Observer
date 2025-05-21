function visualize_robot(X, X_ref, k, params)
    % Visualize the Mecanum robot in its current state
    
    % Extract positions
    x = X(1, k);
    y = X(2, k);
    theta = X(3, k);
    
    x_ref = X_ref(1, k);
    y_ref = X_ref(2, k);
    theta_ref = X_ref(3, k);
    
    % Robot dimensions
    lx = params.lx;
    ly = params.ly;
    r = params.r;
    
    % Robot body corners in robot frame
    corners_robot = [lx, ly; -lx, ly; -lx, -ly; lx, -ly; lx, ly]';
    
    % Rotation matrix
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    R_ref = [cos(theta_ref), -sin(theta_ref); sin(theta_ref), cos(theta_ref)];
    
    % Robot body corners in global frame
    corners_global = R * corners_robot;
    corners_global_x = corners_global(1, :) + x;
    corners_global_y = corners_global(2, :) + y;
    
    corners_ref_global = R_ref * corners_robot;
    corners_ref_global_x = corners_ref_global(1, :) + x_ref;
    corners_ref_global_y = corners_ref_global(2, :) + y_ref;
    
    % Plot
    clf;
    hold on;
    
    % Plot reference trajectory (past and future)
    plot(X_ref(1, 1:end), X_ref(2, 1:end), 'b--', 'LineWidth', 1);
    
    % Plot actual trajectory (past)
    plot(X(1, 1:k), X(2, 1:k), 'r-', 'LineWidth', 1);
    
    % Plot reference robot
    fill(corners_ref_global_x, corners_ref_global_y, 'b', 'FaceAlpha', 0.3);
    
    % Plot actual robot
    fill(corners_global_x, corners_global_y, 'r', 'FaceAlpha', 0.5);
    
    % Plot wheels (simplified)
    wheel_centers = [
        [lx, ly];
        [-lx, ly];
        [-lx, -ly];
        [lx, -ly]
    ];
    
    for i = 1:4
        wheel_center_robot = wheel_centers(i, :)';
        wheel_center_global = R * wheel_center_robot + [x; y];
        
        wheel_center_ref_global = R_ref * wheel_center_robot + [x_ref; y_ref];
        
        % Draw actual wheel
        rectangle('Position', [wheel_center_global(1)-r/2, wheel_center_global(2)-r/2, r, r], ...
                 'Curvature', [1, 1], 'FaceColor', 'k');
        
        % Draw reference wheel
        rectangle('Position', [wheel_center_ref_global(1)-r/2, wheel_center_ref_global(2)-r/2, r, r], ...
                 'Curvature', [1, 1], 'FaceColor', 'b', 'FaceAlpha', 0.3);
    end
    
    % Draw direction vector
    arrow_length = 0.3;
    quiver(x, y, arrow_length*cos(theta), arrow_length*sin(theta), 0, 'r', 'LineWidth', 2);
    quiver(x_ref, y_ref, arrow_length*cos(theta_ref), arrow_length*sin(theta_ref), 0, 'b', 'LineWidth', 2);
    
    % Set plot properties
    grid on;
    axis equal;
    xlim([min(X_ref(1, :))-1, max(X_ref(1, :))+1]);
    ylim([min(X_ref(2, :))-1, max(X_ref(2, :))+1]);
    title(sprintf('Time: %.2f s', (k-1)*0.01));
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    legend('Reference Trajectory', 'Actual Trajectory', 'Reference Robot', 'Actual Robot', 'Location', 'best');
    
    drawnow;
end
