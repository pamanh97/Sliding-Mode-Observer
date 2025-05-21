function metrics = calculate_performance_metrics(X, X_ref, U, sim_params)
    % Calculate performance metrics for evaluation
    
    N = sim_params.N;
    
    % Root Mean Square Error (RMSE)
    squared_errors = zeros(1, N);
    for k = 1:N
        error_vec = X(1:3, k) - X_ref(1:3, k);
        squared_errors(k) = error_vec' * error_vec;
    end
    metrics.RMSE = sqrt(mean(squared_errors));
    
    % Control Effort
    control_effort = 0;
    for k = 1:N
        control_effort = control_effort + sum(abs(U(:, k)));
    end
    metrics.CE = control_effort / N;
    
    % Maximum tracking error
    max_error_sq = 0;
    for k = 1:N
        error_vec = X(1:3, k) - X_ref(1:3, k);
        error_sq = error_vec' * error_vec;
        if error_sq > max_error_sq
            max_error_sq = error_sq;
        end
    end
    metrics.E_max = sqrt(max_error_sq);
    
    % Settling time (time to reach error < 0.05m)
    settling_threshold = 0.05^2;
    metrics.Ts = sim_params.T_final;  % Default if never settles
    for k = 1:N
        error_vec = X(1:3, k) - X_ref(1:3, k);
        error_sq = error_vec' * error_vec;
        if error_sq < settling_threshold
            metrics.Ts = sim_params.t(k);
            break;
        end
    end
    
    % Display metrics
    fprintf('Performance Metrics:\n');
    fprintf('RMSE: %.4f\n', metrics.RMSE);
    fprintf('Control Effort: %.4f\n', metrics.CE);
    fprintf('Maximum Error: %.4f\n', metrics.E_max);
    fprintf('Settling Time: %.4f s\n', metrics.Ts);
end
