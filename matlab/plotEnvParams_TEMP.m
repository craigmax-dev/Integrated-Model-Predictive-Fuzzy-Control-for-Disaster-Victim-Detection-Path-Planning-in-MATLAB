% close all
% clear all
% 
% % Set up the environment
% n_x_e = 9;  % Number of cells in x direction
% n_y_e = 9;  % Number of cells in y direction
% n_x_e = n_x_e;
% n_y_e = n_y_e;
% 
% % Fire setup
% m_bt = 200 .* ones(n_x_e, n_y_e);
% m_f = ones(n_x_e, n_y_e);  % Initialize the fire map with no fires
% m_f(5, 5) = 3;  % Set cells [5:6,5:6] as active fire (fire spreading)
% 
% % Initialize series matrices to store fire and downwind map states
% m_f_series = zeros(n_x_e, n_y_e, 2);  % Store for current and next step
% m_dw_e_series = zeros(n_x_e, n_y_e, 2);
% 
% % Parameters for wind model
% r_w = 3;
% c_wm_1 = 0.2;
% c_wm_2 = 0.3;
% c_wm_d = 0.9;
% ang_w = pi / 4;  % Wind angle (e.g., NE direction)
% 
% % MANUALLY ADDED - CORRECT?
% c_fs_1 = 1;
% c_fs_2 = 1;
% m_s = ones(n_x_e, n_y_e);
% m_bo = ones(n_x_e, n_y_e);
% 
% % Burnout and ignition time
% t_i = 120;  % Ignition time in seconds
% t_b = 600;  % Burnout time in seconds
% 
% % Time step for simulation
% dt_e = 15;  % Time step size in seconds
% k_e = 1;  % Simulation step
% 
% % Wind speeds to test
% wind_speeds = [0, 1, 3];
% 
% % Prepare separate figures for each parameter
% figure_F = figure('Name', 'Fire Spread');
% set(gcf, 'Position', [0, 100, length(wind_speeds)*300, 300]);  % [left, bottom, width, height]
% 
% figure_W = figure('Name', 'Wind Spread');
% set(gcf, 'Position', [0, 600, length(wind_speeds)*300, 300]);  % [left, bottom, width, height]
% 
% figure_dw_e = figure('Name', 'Downwind Map');
% set(gcf, 'Position', [length(wind_speeds)*300+100, 100, length(wind_speeds)*300, 300]);  % [left, bottom, width, height]
% 
% figure_m_f = figure('Name', 'Fire Map');
% set(gcf, 'Position', [length(wind_speeds)*300+100, 600, length(wind_speeds)*300, 300]);  % [left, bottom, width, height]
% 
% % Loop through each wind speed
% for idx_plt = 1:length(wind_speeds)
%     v_w = wind_speeds(idx_plt);  % Set wind speed
% 
% 
%     %% DOWNWIND MAP
%     % Initialize m_dw_e with zeros
%     m_dw_e = zeros(n_x_e, n_y_e);
% 
%     for i = 1:n_x_e
%         for j = 1:n_y_e
%             if m_f(i, j) == 2 || m_f(i, j) == 3  % Active or burning fire
% 
%                 [X, Y] = meshgrid(1:n_x_e, 1:n_y_e);
%                 F_d = atan2(Y - i, X - j);
%                 ang_diff = ang_w - F_d;
%                 W_dir_dw = exp(v_w * (c_wm_1 + c_wm_2 * (cos(ang_diff) - 1)));
%                 W_dir_dw = mat2gray(W_dir_dw);  % Normalize wind direction influence
% 
%                 W_dis_dw = 1 - sqrt((X - i).^2 + (Y - j).^2) / sqrt(n_x_e^2 + n_y_e^2);
% 
%                 m_dw_e = max(m_dw_e, W_dir_dw .* W_dis_dw);
%             end
%         end
%     end
% 
%     m_dw_e = 1 - m_dw_e;  % Invert the downwind effect
%     % % Call the environment model function to update fire states and calculate spread
%     % environment_model = model_environment(environment_model, k_e, dt_e);
% 
% 
%    %%%%%%%%%%%%%%%%%%%%%%%% CALC W
%     [X, Y] = meshgrid(-r_w:r_w, -r_w:r_w);
% 
%     % Calculate angle difference between wind direction and cell direction
%     F_d = atan2(Y, X);
%     ang_diff = ang_w - F_d;
% 
%     % Calculate wind direction modifier using vectorized operations
%     w_dir = exp(v_w * (c_wm_1 + c_wm_2 * (cos(ang_diff) - 1)));
% 
%     % Calculate wind distance modifier using vectorized operations
%     distances = sqrt(X.^2 + Y.^2);
%     w_dis = c_wm_d .^ distances;
% 
%     % Combine direction and distance modifiers
%     W = w_dir .* w_dis;
% 
%     %%%%%%%%%%%%%%%%%%%%%%%%% CALC F
%     % Initialize F with the correct dimensions
%     F = zeros(n_x_e, n_y_e);
% 
%     % Direct manipulation of m_bt and m_f
%     m_bt = m_bt;
%     m_f = m_f;
% 
%     % Logical indexing and update
%     active_or_burning = (m_f == 2) | (m_f == 3);
%     m_bt(active_or_burning) = m_bt(active_or_burning) + dt_e;  % Correctly increments the burn time for active or burning cells by dt_e
% 
%     % Transition from active to burning state based on ignition time
%     m_f(m_f == 2 & m_bt >= t_i) = 3;
% 
%     % Update fire spread probability for burning cells and transition to burnout state
%     burning_cells = find(m_f == 3);
%     for idx = 1:length(burning_cells)
% 
%         [i, j] = ind2sub([n_x_e, n_y_e], burning_cells(idx));
% 
%         %% CALC P
%         % Calculation of fire spread probability based on burn time
%         if m_bt(i, j) <= (t_b - t_i) / 5 + t_i
%             p = 4 / (t_b - t_i) * m_bt(i, j) + (0.2 * t_b - 4.2 * t_i) / (t_b - t_i);
%         elseif m_bt(i, j) <= t_b
%             p = 5 / (4 * (t_b - t_i)) * (-m_bt(i, j) + t_b);
%         else
%             p = 0; % No spread if outside of time range
%         end
% 
%         %% CALC F
%         % Determine the range for local update
%         row_range = max(1, i - r_w) : min(n_x_e, i + r_w);
%         col_range = max(1, j - r_w) : min(n_y_e, j + r_w);
% 
%         % Adjust indices in W to align with F's indices
%         W_row_range = (row_range - i + r_w + 1);
%         W_col_range = (col_range - j + r_w + 1);
% 
%         % Update the fire spread probability
%         local_W = W(W_row_range, W_col_range);
%         F(row_range, col_range) = F(row_range, col_range) + ...
%                                   c_fs_1 * (m_s(row_range, col_range) .* m_bo(row_range, col_range)) .* ...
%                                   local_W.^c_fs_2 * p;
% 
%     end    
% 
%     % Calculate the common color scale limits across all wind speeds
%     min_F = min(F(:));  % Minimum value in F
%     max_F = max(F(:));  % Maximum value in F
%     min_W = min(W(:));  % Minimum value in W
%     max_W = max(W(:));  % Maximum value in W
%     min_dw_e = min(m_dw_e(:));  % Minimum value in m_dw_e
%     max_dw_e = max(m_dw_e(:));  % Maximum value in m_dw_e
%     min_m_f = min(m_f(:));  % Minimum value in m_dw_e
%     max_m_f = max(m_f(:));  % Maximum value in m_dw_e
% 
%     % Plot Fire Spread (F)
%     figure(figure_F);
%     subplot(1, length(wind_speeds), idx_plt);
%     imagesc(F);
%     colorbar;
%     axis equal tight;
%     set(gca, 'YDir', 'normal');  % Reverse the y-axis
%     xlabel('$x$ cell index', 'Interpreter', 'latex');
%     ylabel('$y$ cell index', 'Interpreter', 'latex');
%     title(['$\mathbf{F}$, $v^{\mathrm{wind}} = ' num2str(v_w) '$ m/s'], 'Interpreter', 'latex');
%     % caxis([min_F max_F]);  % Set common color scale for F
%     caxis([0 1.5]);  % Set common color scale for F
% 
%     % Plot Wind Spread (W)
%     figure(figure_W);
%     subplot(1, length(wind_speeds), idx_plt);
%     imagesc(W);
%     colorbar;
%     axis equal tight;
%     set(gca, 'YDir', 'normal');  % Reverse the y-axis
%     xlabel('$x$ cell index', 'Interpreter', 'latex');
%     ylabel('$y$ cell index', 'Interpreter', 'latex');
%     title(['$\mathbf{W}$, $v^{\mathrm{wind}} = ' num2str(v_w) '$ m/s'], 'Interpreter', 'latex');
%     % caxis([min_W max_W]);  % Set common color scale for W
%     caxis([0 1.5]);  % Set common color scale for F
% 
%     % Plot Downwind Map (m_dw_e)
%     figure(figure_dw_e);
%     subplot(1, length(wind_speeds), idx_plt);
%     imagesc(m_dw_e);
%     colorbar;
%     axis equal tight;
%     set(gca, 'YDir', 'normal');  % Reverse the y-axis
%     xlabel('$x$ cell index', 'Interpreter', 'latex');
%     ylabel('$y$ cell index', 'Interpreter', 'latex');
%     title(['$\mathbf{M}^{\mathrm{downwind}}$, $v^{\mathrm{wind}} = ' num2str(v_w) '$ m/s'], 'Interpreter', 'latex');
%     % caxis([min_dw_e max_dw_e]);  % Set common color scale for m_dw_e
%     caxis([0 1]);  % Set common color scale for F
% 
%     % Plot Downwind Map (m_dw_e)
%     figure(figure_m_f);
%     subplot(1, length(wind_speeds), idx_plt);
%     imagesc(m_f);
%     colorbar;
%     axis equal tight;
%     set(gca, 'YDir', 'normal');  % Reverse the y-axis
%     xlabel('$x$ cell index', 'Interpreter', 'latex');
%     ylabel('$y$ cell index', 'Interpreter', 'latex');
%     title(['$\mathbf{M}^{\mathrm{fire}}$, $v^{\mathrm{wind}} = ' num2str(v_w) '$ m/s'], 'Interpreter', 'latex');
%     % caxis([min_m_f max_m_f]);  % Set common color scale for m_dw_e
%     caxis([0 3]);  % Set common color scale for F
% end
% 
% Set the overall figure titles
% sgtitle(figure_F, 'Fire Spread for Different Wind Speeds');
% sgtitle(figure_W, 'Wind Spread for Different Wind Speeds');
% sgtitle(figure_dw_e, 'Downwind Map for Different Wind Speeds');
% 
% %% Plot p with bt
% 
% % % Define parameters
% % t_i = 120;  % Ignition time in seconds
% % t_b = 600;  % Burnout time in seconds
% % 
% % % Generate a range of burn times from 0 to t_b
% % m_bt_values = linspace(0, t_b, 1000);  % 1000 values between 0 and t_b
% % p_values = zeros(size(m_bt_values));  % Preallocate the p values
% % 
% % % Compute p for each burn time (m_bt)
% % for k = 1:length(m_bt_values)
% %     m_bt = m_bt_values(k);
% % 
% %     if m_bt <= (t_b - t_i) / 5 + t_i
% %         p_values(k) = 4 / (t_b - t_i) * m_bt + (0.2 * t_b - 4.2 * t_i) / (t_b - t_i);
% %     elseif m_bt <= t_b
% %         p_values(k) = 5 / (4 * (t_b - t_i)) * (-m_bt + t_b);
% %     else
% %         p_values(k) = 0;  % No spread if outside of time range
% %     end
% % end
% % 
% % % Plot the relationship between burn time (m_bt) and fire spread probability (p)
% % figure;
% % plot(m_bt_values, p_values, 'LineWidth', 2);
% % xlabel('Burn Time, $t^{\mathrm{burn}} (s)$', 'Interpreter', 'latex');
% % ylabel('Ignition Probability, $p^{\mathrm{ignition}}$', 'Interpreter', 'latex');
% % % title('Relationship Between Burn Time and Fire Spread Probability', 'Interpreter', 'latex');
% % grid on;
% 
% % %% PLOT Agent movement time
% % close all;
% % clear all;
% % 
% % % Set up the environment
% % n_x_e = 9;  % Number of cells in x direction
% % n_y_e = 9;  % Number of cells in y direction
% % start_loc = [5, 5];  % Agent starting location
% % 
% % % Parameters for agent dynamics and wind model
% % l_x_s = 100;  % Length of each cell in x direction (meters)
% % l_y_s = 100;  % Length of each cell in y direction (meters)
% % v_as = 5;    % Agent airspeed (m/s) 
% % ang_w = pi / 4;  % Wind angle (e.g., NE direction)
% % wind_speeds = [0, 1, 3];  % Wind speeds to test (m/s)
% % 
% % % Prepare figure for plotting
% % figure_travel_times = figure('Name', 'Agent Travel Times');
% % set(gcf, 'Position', [100, 100, length(wind_speeds) * 300, 300]);  % [left, bottom, width, height]
% % 
% % % Loop through each wind speed and calculate travel times
% % for idx_plt = 1:length(wind_speeds)
% %     v_w = wind_speeds(idx_plt);  % Set wind speed
% %     travel_times = zeros(n_x_e, n_y_e);  % Initialize matrix to store travel times
% % 
% %     % Calculate travel times for each destination cell in the environment
% %     for i = 1:n_x_e
% %         for j = 1:n_y_e
% %             dest_loc = [i, j];  % Destination location
% %             travel_times(i, j) = calc_t_trav(start_loc, dest_loc, l_x_s, l_y_s, ang_w, v_w, v_as);
% %         end
% %     end
% % 
% %     % Plot the travel times
% %     figure(figure_travel_times);
% %     subplot(1, length(wind_speeds), idx_plt);
% %     imagesc(travel_times);
% %     colorbar;
% %     axis equal tight;
% %     set(gca, 'YDir', 'normal');  % Reverse the y-axis
% % 
% %     % Plot gridlines
% %     hold on;
% %     for x = 0.5:n_x_e+0.5
% %         plot([0.5, n_y_e+0.5], [x, x], 'k', 'LineWidth', 1);  % Horizontal lines
% %     end
% %     for y = 0.5:n_y_e+0.5
% %         plot([y, y], [0.5, n_x_e+0.5], 'k', 'LineWidth', 1);  % Vertical lines
% %     end
% % 
% %     % Plot the agent position
% %     plot(start_loc(2), start_loc(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'r');
% %     hold off;
% % 
% %     xlabel('$x$ cell index', 'Interpreter', 'latex');
% %     ylabel('$y$ cell index', 'Interpreter', 'latex');
% %     title(['$t^{\mathrm{travel}}$, $v^{\mathrm{wind}} = ' num2str(v_w) '$ m/s'], 'Interpreter', 'latex');
% % 
% %     % % Set color axis limits to be the same for all subplots
% %     % caxis([0 max(travel_times(:))]);  % Set common color scale based on max travel time
% %     caxis([0 250]);  % Set common color scale based on max travel time
% % end
% 
% % 
% % sgtitle('Agent Travel Times for Different Wind Speeds');
% 
% % % %% PLOT FIRE SPREAD AND STRUCTURE / WIND RELATIONSHIP
% % function x = plotEnvParams_TEMP()
% %     close all
% %     clear all
% % 
% %     % Simulation parameters
% %     timesteps = [1, 21, 41];  % Time steps to simulate
% %     structure_values = [0.25, 0.5, 1];  % Structure map values to test
% %     wind_values = [0, 1, 3];  % Different wind speeds to simulate
% % 
% %     dt_e = 15;  % Time step in seconds
% %     start_fire = [11, 11];  % Initial fire position
% % 
% %     % Prepare figure for plotting
% %     figure_fire_spread = figure('Name', 'Fire Spread Over Time');
% %     set(gcf, 'Position', [100, 100, 900, 900]);  % [left, bottom, width, height]
% % 
% %     % Loop through each structure map and simulate fire spread
% %     for idx = 1:length(wind_values)
% % 
% %         config = i_sim_comms_disabled;
% %         environment_model = i_env_dynamics_20(config);  % Initialize environment
% % 
% %         structure_value = 0.5;  % Set structure map value
% % 
% %         % Set wind speed for the current simulation
% %         environment_model.v_w = wind_values(idx);  % Set wind speed
% %         environment_model.ang_w = pi / 4;          % Wind angle (45 degrees, NE)
% %         environment_model.c_fs_1 = 0.1;            % Increase base fire spread probability
% %         environment_model.c_fs_2 = 1.2;              % Further increase wind sensitivity
% %         environment_model.c_wm_1 = 0.15;           % Increase wind direction influence
% %         environment_model.c_wm_2 = 1;               % Keep angle scaling with wind
% %         environment_model.c_wm_d = 1;           % Increase wind influence across distance
% % 
% %         % Create structure map with the specified value
% %         environment_model.m_s = ones(environment_model.n_x_e, environment_model.n_y_e) * structure_value;
% % 
% %         % Set initial fire
% %         environment_model.m_f = ones(environment_model.n_x_e, environment_model.n_y_e);
% %         environment_model.m_f(start_fire(1), start_fire(2)) = 3;  % Start fire
% %         environment_model.m_f_series(:, :, 1) = environment_model.m_f;
% % 
% %         % Simulate fire spread over time
% %         for k_e = 1:max(timesteps)
% %             % Update environment model with fire spread simulation
% %             environment_model = model_environment(environment_model, k_e, dt_e);
% % 
% %             % Plot fire map for selected timesteps
% %             if ismember(k_e, timesteps)
% %                 plot_idx = (idx - 1) * length(timesteps) + find(timesteps == k_e);
% %                 subplot(length(structure_values), length(timesteps), plot_idx);
% % 
% %                 fire_map = environment_model.m_f_series(:, :, k_e);
% %                 imagesc(fire_map);
% %                 colorbar;
% %                 axis equal tight;
% %                 set(gca, 'YDir', 'normal');  % Reverse the y-axis
% %                 title(['$k = ' num2str(k_e) ', v^{\mathrm{wind}} = ' num2str(wind_values(idx)) 'm/s$'], 'Interpreter', 'latex');
% %                 caxis([1 4]);  % Fire state from 1 to 4
% %             end
% %         end
% %     end
% %     % sgtitle('Fire Spread Over Time for Different Structure Maps');
% % end
% 
% %% FIS INPUT/OUTPUT PLOT PREPARATION
% % % Plot for fisInputs.cell_fire_time_risk
% % figure;
% % imagesc(fisInputs.cell_fire_time_risk);
% % colorbar;
% % axis equal tight;
% % set(gca, 'YDir', 'normal');
% % 
% % % Plot for fisInputs.cell_scan_certainty
% % figure;
% % imagesc(fisInputs.cell_scan_certainty);
% % colorbar;
% % axis equal tight;
% % set(gca, 'YDir', 'normal');
% % 
% % % Plot for fisInputs.cell_priority
% % figure;
% % imagesc(fisInputs.cell_priority);
% % colorbar;
% % axis equal tight;
% % set(gca, 'YDir', 'normal');
% % 
% % % Plot for fisInputs.t_response
% % figure;
% % imagesc(fisInputs.t_response);
% % colorbar;
% % axis equal tight;
% % set(gca, 'YDir', 'normal');
% % 
% % % Plot for agent_model.m_bo_s (Gaussian distribution)
% % figure;
% % imagesc(agent_model.m_bo_s);
% % colorbar;
% % axis equal tight;
% % set(gca, 'YDir', 'normal');
% % 
% % % Plot for FIS Output (m_att: attraction map)
% % figure;
% % imagesc(m_att);
% % colorbar;
% % axis equal tight;
% % set(gca, 'YDir', 'normal');
% 
% 
