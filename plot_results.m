function plot_results(X, X_ref, U, U_mpc, U_pid, beta, e_p, sim_params)
    % Plot simulation results
    
    t = sim_params.t;
    
    % Figure 1: Trajectory Tracking
    figure(1);
    plot(X_ref(1,:), X_ref(2,:), 'b--', 'LineWidth', 2);
    hold on;
    plot(X(1,:), X(2,:), 'r-', 'LineWidth', 1.5);
    grid on;
    legend('Reference', 'Actual', 'Location', 'best');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Trajectory Tracking Performance');
    
    % Figure 2: Tracking Errors
    figure(2);
    subplot(3,1,1);
    plot(t, X(1,:) - X_ref(1,:), 'b-', 'LineWidth', 1.5);
    grid on;
    ylabel('X Error (m)');
    title('Position Tracking Errors');
    
    subplot(3,1,2);
    plot(t, X(2,:) - X_ref(2,:), 'r-', 'LineWidth', 1.5);
    grid on;
    ylabel('Y Error (m)');
    
    subplot(3,1,3);
    plot(t, X(3,:) - X_ref(3,:), 'g-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('θ Error (rad)');
    
    % Figure 3: Control Inputs
    figure(3);
    subplot(2,2,1);
    plot(t, U(1,:), 'b-', 'LineWidth', 1.5);
    grid on;
    ylabel('Voltage (V)');
    title('Motor 1 Control Input');
    
    subplot(2,2,2);
    plot(t, U(2,:), 'r-', 'LineWidth', 1.5);
    grid on;
    ylabel('Voltage (V)');
    title('Motor 2 Control Input');
    
    subplot(2,2,3);
    plot(t, U(3,:), 'g-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('Voltage (V)');
    title('Motor 3 Control Input');
    
    subplot(2,2,4);
    plot(t, U(4,:), 'm-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('Voltage (V)');
    title('Motor 4 Control Input');
    
    % Figure 4: Controller Integration
    figure(4);
    subplot(2,1,1);
    plot(t, beta, 'b-', 'LineWidth', 1.5);
    grid on;
    ylabel('β');
    title('Control Blending Factor (MPC weight)');
    ylim([0 1]);
    
    subplot(2,1,2);
    plot(t, e_p, 'r-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    ylabel('Error (m)');
    title('Position Error Magnitude');
    
    % Figure 5: Control Comparison
    figure(5);
    subplot(2,2,1);
    plot(t, U_mpc(1,:), 'b-', t, U_pid(1,:), 'r--', t, U(1,:), 'k-', 'LineWidth', 1.5);
    grid on;
    legend('MPC', 'PID', 'Hybrid', 'Location', 'best');
    title('Motor 1 Control Comparison');
    
    subplot(2,2,2);
    plot(t, U_mpc(2,:), 'b-', t, U_pid(2,:), 'r--', t, U(2,:), 'k-', 'LineWidth', 1.5);
    grid on;
    title('Motor 2 Control Comparison');
    
    subplot(2,2,3);
    plot(t, U_mpc(3,:), 'b-', t, U_pid(3,:), 'r--', t, U(3,:), 'k-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    title('Motor 3 Control Comparison');
    
    subplot(2,2,4);
    plot(t, U_mpc(4,:), 'b-', t, U_pid(4,:), 'r--', t, U(4,:), 'k-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time (s)');
    title('Motor 4 Control Comparison');
    
    % Figure 6: 3D Visualization
    figure(6);
    subplot(3,1,1);
    plot3(X_ref(1,:), X_ref(2,:), X_ref(3,:), 'b--', 'LineWidth', 2);
    hold on;
    plot3(X(1,:), X(2,:), X(3,:), 'r-', 'LineWidth', 1.5);
    grid on;
    legend('Reference', 'Actual', 'Location', 'best');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    zlabel('θ (rad)');
    title('3D Trajectory Visualization');
    
    subplot(3,1,2);
    plot(t, X(4,:), 'b-', t, X_ref(4,:), 'b--', ...
         t, X(5,:), 'r-', t, X_ref(5,:), 'r--', ...
         'LineWidth', 1.5);
    grid on;
    legend('x\_dot', 'x\_dot\_ref', 'y\_dot', 'y\_dot\_ref', 'Location', 'best');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Linear Velocities');
    
    subplot(3,1,3);
    plot(t, X(6,:), 'g-', t, X_ref(6,:), 'g--', 'LineWidth', 1.5);
    grid on;
    legend('θ\_dot', 'θ\_dot\_ref', 'Location', 'best');
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    title('Angular Velocity');
end
