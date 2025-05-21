function [X_ref, V_ref] = generate_reference_trajectory(trajectory_type, t)
    % Generate reference trajectory based on the selected type
    
    N = length(t);
    X_ref = zeros(6, N);  % State references [x, y, θ, x_dot, y_dot, θ_dot]
    V_ref = zeros(3, N);  % Velocity references [v_x, v_y, ω]
    
    switch trajectory_type
        case 1  % Circular trajectory
            for i = 1:N
                X_ref(1, i) = 2 * cos(0.2 * t(i));             % x
                X_ref(2, i) = 2 * sin(0.2 * t(i));             % y
                X_ref(3, i) = 0.2 * t(i);                      % θ
                X_ref(4, i) = -2 * 0.2 * sin(0.2 * t(i));      % x_dot
                X_ref(5, i) = 2 * 0.2 * cos(0.2 * t(i));       % y_dot
                X_ref(6, i) = 0.2;                             % θ_dot
                
                V_ref(1, i) = -2 * 0.2 * sin(0.2 * t(i));      % v_x
                V_ref(2, i) = 2 * 0.2 * cos(0.2 * t(i));       % v_y
                V_ref(3, i) = 0.2;                             % ω
            end
            
        case 2  % Square trajectory with rounded corners
            for i = 1:N
                time = t(i);
                if time < 10
                    X_ref(1, i) = time - 5;
                    X_ref(2, i) = -5;
                    X_ref(3, i) = pi/4;
                    X_ref(4, i) = 1;
                    X_ref(5, i) = 0;
                elseif time < 20
                    X_ref(1, i) = 5;
                    X_ref(2, i) = -5 + (time - 10);
                    X_ref(3, i) = pi/4;
                    X_ref(4, i) = 0;
                    X_ref(5, i) = 1;
                elseif time < 30
                    X_ref(1, i) = 5 - (time - 20);
                    X_ref(2, i) = 5;
                    X_ref(3, i) = pi/4;
                    X_ref(4, i) = -1;
                    X_ref(5, i) = 0;
                else
                    X_ref(1, i) = -5;
                    X_ref(2, i) = 5 - (time - 30);
                    X_ref(3, i) = pi/4;
                    X_ref(4, i) = 0;
                    X_ref(5, i) = -1;
                end
                X_ref(6, i) = 0;
                
                V_ref(1, i) = X_ref(4, i);
                V_ref(2, i) = X_ref(5, i);
                V_ref(3, i) = 0;
            end
            
        case 3  % Figure-eight trajectory
            for i = 1:N
                X_ref(1, i) = 3 * sin(0.1 * t(i));                          % x
                X_ref(2, i) = 2 * sin(0.2 * t(i));                          % y
                X_ref(4, i) = 3 * 0.1 * cos(0.1 * t(i));                    % x_dot
                X_ref(5, i) = 2 * 0.2 * cos(0.2 * t(i));                    % y_dot
                X_ref(3, i) = atan2(X_ref(5, i), X_ref(4, i));              % θ
                X_ref(6, i) = (X_ref(4, i)*2*0.2*(-sin(0.2*t(i))) - X_ref(5, i)*3*0.1*(-sin(0.1*t(i)))) / (X_ref(4, i)^2 + X_ref(5, i)^2); % θ_dot
                
                V_ref(1, i) = X_ref(4, i);
                V_ref(2, i) = X_ref(5, i);
                V_ref(3, i) = X_ref(6, i);
            end
            
        case 4  % S-curve trajectory
            for i = 1:N
                X_ref(1, i) = 0.5 * t(i);                                  % x
                X_ref(2, i) = 2 * tanh(t(i) - 10);                         % y
                X_ref(4, i) = 0.5;                                          % x_dot
                X_ref(5, i) = 2 * (1 - tanh(t(i) - 10)^2);                 % y_dot
                X_ref(3, i) = atan2(X_ref(5, i), X_ref(4, i));             % θ
                X_ref(6, i) = -2 * 2 * tanh(t(i) - 10) * (1 - tanh(t(i) - 10)^2) / (X_ref(4, i)^2 + X_ref(5, i)^2); % θ_dot
                
                V_ref(1, i) = X_ref(4, i);
                V_ref(2, i) = X_ref(5, i);
                V_ref(3, i) = X_ref(6, i);
            end
    end
end
