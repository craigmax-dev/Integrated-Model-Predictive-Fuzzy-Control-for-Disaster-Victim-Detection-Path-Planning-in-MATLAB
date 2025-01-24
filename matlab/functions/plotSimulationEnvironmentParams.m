function plotSimulationEnvironmentParams()
    % PLOTSIMULATIONENVIRONMENTPARAMS
    % This function generates the environment parameter figures used in the
    % thesis writeup.
    close all

    % Set up the environment
    n_x_e = 9;  % Number of cells in x direction
    n_y_e = 9;  % Number of cells in y direction

    % Fire setup
    m_bt = 200 .* ones(n_x_e, n_y_e);
    m_f = ones(n_x_e, n_y_e);  % Initialize the fire map with no fires
    m_f(5, 5) = 3;             % Set a cell as active fire

    % Initialize series matrices to store fire and downwind map states
    m_f_series = zeros(n_x_e, n_y_e, 2);  
    m_dw_e_series = zeros(n_x_e, n_y_e, 2);

    % Parameters for wind model
    r_w = 3;
    c_wm_1 = 0.2;
    c_wm_2 = 0.3;
    c_wm_d = 0.9;
    ang_w = pi / 4;  % Wind angle (e.g., NE direction)

    % Additional parameters for fire spread
    c_fs_1 = 1;
    c_fs_2 = 1;
    m_s = ones(n_x_e, n_y_e);
    m_bo = ones(n_x_e, n_y_e);

    % Burnout and ignition time
    t_i = 120;  % Ignition time in seconds
    t_b = 600;  % Burnout time in seconds

    % Time step for simulation
    dt_e = 15;  % Time step size in seconds
    k_e = 1;    % Simulation step

    % Wind speeds to test
    wind_speeds = [0, 1, 3];

    % Prepare separate figures for each parameter
    figure_F = figure('Name', 'Fire Spread');
    set(gcf, 'Position', [0, 100, length(wind_speeds)*300, 300]);

    figure_W = figure('Name', 'Wind Spread');
    set(gcf, 'Position', [0, 600, length(wind_speeds)*300, 300]);

    figure_dw_e = figure('Name', 'Downwind Map');
    set(gcf, 'Position', [length(wind_speeds)*300+100, 100, length(wind_speeds)*300, 300]);

    figure_m_f = figure('Name', 'Fire Map');
    set(gcf, 'Position', [length(wind_speeds)*300+100, 600, length(wind_speeds)*300, 300]);

    % Define a custom discrete colormap for the fire map
    myCmap = [
        0.8,   0.8,   0.8;    % 0: non-flammable (light gray)
        0.9,   0.95,  1.0;    % 1: flammable (pale blue)
        1.0,   0.498, 0.0;    % 2: catching fire (orange)
        0.894, 0.102, 0.110;  % 3: burning (red)
        0.0,   0.0,   0.0     % 4: extinguished (black)
    ];

    % Loop through each wind speed
    for idx_plt = 1:length(wind_speeds)
        v_w = wind_speeds(idx_plt);  % Set wind speed

        %% DOWNWIND MAP
        m_dw_e = zeros(n_x_e, n_y_e);
        for i = 1:n_x_e
            for j = 1:n_y_e
                if m_f(i, j) == 2 || m_f(i, j) == 3  % Active or burning fire
                    [X, Y] = meshgrid(1:n_x_e, 1:n_y_e);
                    F_d = atan2(Y - i, X - j);
                    ang_diff = ang_w - F_d;
                    W_dir_dw = exp(v_w * (c_wm_1 + c_wm_2 * (cos(ang_diff) - 1)));
                    W_dir_dw = mat2gray(W_dir_dw);  % Normalize wind direction influence

                    W_dis_dw = 1 - sqrt((X - i).^2 + (Y - j).^2) / sqrt(n_x_e^2 + n_y_e^2);

                    m_dw_e = max(m_dw_e, W_dir_dw .* W_dis_dw);
                end
            end
        end
        m_dw_e = 1 - m_dw_e;  % Invert the downwind effect

        %%%%%%%%%%%%%%%%%%%%%%%% CALC W
        [X, Y] = meshgrid(-r_w:r_w, -r_w:r_w);

        % Calculate angle difference between wind direction and cell direction
        F_d = atan2(Y, X);
        ang_diff = ang_w - F_d;

        % Calculate wind direction modifier using vectorized operations
        w_dir = exp(v_w * (c_wm_1 + c_wm_2 * (cos(ang_diff) - 1)));

        % Calculate wind distance modifier using vectorized operations
        distances = sqrt(X.^2 + Y.^2);
        w_dis = c_wm_d .^ distances;

        % Combine direction and distance modifiers
        W = w_dir .* w_dis;

        %%%%%%%%%%%%%%%%%%%%%%%%% CALC F
        % Initialize F with the correct dimensions
        F = zeros(n_x_e, n_y_e);

        % Logical indexing and update
        active_or_burning = (m_f == 2) | (m_f == 3);
        m_bt(active_or_burning) = m_bt(active_or_burning) + dt_e;  % Increment burn time

        % Transition from active to burning state based on ignition time
        m_f(m_f == 2 & m_bt >= t_i) = 3;

        % Update fire spread probability for burning cells
        burning_cells = find(m_f == 3);
        for idx = 1:length(burning_cells)
            [i, j] = ind2sub([n_x_e, n_y_e], burning_cells(idx));

            %% CALC P (Ignition probability)
            if m_bt(i, j) <= (t_b - t_i) / 5 + t_i
                p = 4 / (t_b - t_i) * m_bt(i, j) + (0.2 * t_b - 4.2 * t_i) / (t_b - t_i);
            elseif m_bt(i, j) <= t_b
                p = 5 / (4 * (t_b - t_i)) * (-m_bt(i, j) + t_b);
            else
                p = 0; % No spread if outside of time range
            end

            %% Update F
            row_range = max(1, i - r_w) : min(n_x_e, i + r_w);
            col_range = max(1, j - r_w) : min(n_y_e, j + r_w);

            W_row_range = (row_range - i + r_w + 1);
            W_col_range = (col_range - j + r_w + 1);

            local_W = W(W_row_range, W_col_range);
            F(row_range, col_range) = F(row_range, col_range) + ...
                c_fs_1 * (m_s(row_range, col_range) .* m_bo(row_range, col_range)) .* ...
                local_W.^c_fs_2 * p;
        end

        % Plot Fire Spread (F)
        figure(figure_F);
        subplot(1, length(wind_speeds), idx_plt);
        imagesc(F);
        colorbar; axis equal tight; set(gca, 'YDir', 'normal');
        xlabel('$x$ cell index', 'Interpreter', 'latex');
        ylabel('$y$ cell index', 'Interpreter', 'latex');
        title(['$\mathbf{F}$, $v^{\mathrm{wind}} = ' num2str(v_w) '$ m/s'], 'Interpreter', 'latex');
        caxis([0 1.5]);  

        % Plot Wind Spread (W)
        figure(figure_W);
        subplot(1, length(wind_speeds), idx_plt);
        imagesc(W);
        colorbar; axis equal tight; set(gca, 'YDir', 'normal');
        xlabel('$x$ cell index', 'Interpreter', 'latex');
        ylabel('$y$ cell index', 'Interpreter', 'latex');
        title(['$\mathbf{W}$, $v^{\mathrm{wind}} = ' num2str(v_w) '$ m/s'], 'Interpreter', 'latex');
        caxis([0 1.5]); 

        % Plot Downwind Map (m_dw_e)
        figure(figure_dw_e);
        subplot(1, length(wind_speeds), idx_plt);
        imagesc(m_dw_e);
        colorbar; axis equal tight; set(gca, 'YDir', 'normal');
        xlabel('$x$ cell index', 'Interpreter', 'latex');
        ylabel('$y$ cell index', 'Interpreter', 'latex');
        title(['$\mathbf{M}^{\mathrm{downwind}}$, $v^{\mathrm{wind}} = ' num2str(v_w) '$ m/s'], 'Interpreter', 'latex');
        caxis([0 1]);  

        % Plot Fire Map (m_f)
        figure(figure_m_f);
        subplot(1, length(wind_speeds), idx_plt);
        imagesc(m_f, [0 4]);
        colormap(gca, myCmap); % Apply the custom colormap
        caxis([0 4]);
        cb = colorbar;
        cb.Ticks = 0:1:4;
        cb.TickLabels = {'Non-flammable', 'Flammable', 'Catching Fire', 'Burning', 'Extinguished'};
        axis equal tight; set(gca, 'YDir', 'normal');
        xlabel('$x$ cell index', 'Interpreter', 'latex');
        ylabel('$y$ cell index', 'Interpreter', 'latex');
        title(['$\mathbf{M}^{\mathrm{fire}}$, $v^{\mathrm{wind}} = ' num2str(v_w) '$ m/s'], 'Interpreter', 'latex');
    end

    % Set the overall figure titles
    sgtitle(figure_F, 'Fire Spread for Different Wind Speeds');
    sgtitle(figure_W, 'Wind Spread for Different Wind Speeds');
    sgtitle(figure_dw_e, 'Downwind Map for Different Wind Speeds');

    %% Plot p vs. burn time (m_bt)
    t_i = 120;  % Ignition time in seconds (redeclare here for clarity)
    t_b = 600;  
    m_bt_values = linspace(0, t_b, 1000);
    p_values = zeros(size(m_bt_values));

    for k = 1:length(m_bt_values)
        m_bt_temp = m_bt_values(k);
        if m_bt_temp <= (t_b - t_i) / 5 + t_i
            p_values(k) = 4 / (t_b - t_i) * m_bt_temp + (0.2 * t_b - 4.2 * t_i) / (t_b - t_i);
        elseif m_bt_temp <= t_b
            p_values(k) = 5 / (4 * (t_b - t_i)) * (-m_bt_temp + t_b);
        else
            p_values(k) = 0;
        end
    end

    figure;
    plot(m_bt_values, p_values, 'LineWidth', 2);
    xlabel('Burn Time, $t^{\mathrm{burn}} (s)$', 'Interpreter', 'latex');
    ylabel('Ignition Probability, $p^{\mathrm{ignition}}$', 'Interpreter', 'latex');
    grid on;

    %% Agent travel times
    n_x_e = 9;  
    n_y_e = 9;  
    start_loc = [5, 5];  
    l_x_s = 100;  
    l_y_s = 100;  
    v_as = 5;   
    ang_w = pi / 4;  
    wind_speeds = [0, 1, 3];  

    figure_travel_times = figure('Name', 'Agent Travel Times');
    set(gcf, 'Position', [100, 100, length(wind_speeds) * 300, 300]);

    for idx_plt = 1:length(wind_speeds)
        v_w = wind_speeds(idx_plt);
        travel_times = zeros(n_x_e, n_y_e);

        for i = 1:n_x_e
            for j = 1:n_y_e
                dest_loc = [i, j];
                travel_times(i, j) = calc_t_trav(start_loc, dest_loc, l_x_s, l_y_s, ang_w, v_w, v_as);
            end
        end

        figure(figure_travel_times);
        subplot(1, length(wind_speeds), idx_plt);
        imagesc(travel_times);
        colorbar; axis equal tight; set(gca, 'YDir', 'normal');
        hold on;
        for x = 0.5:n_x_e+0.5
            plot([0.5, n_y_e+0.5], [x, x], 'k', 'LineWidth', 1);
        end
        for y = 0.5:n_y_e+0.5
            plot([y, y], [0.5, n_x_e+0.5], 'k', 'LineWidth', 1);
        end
        plot(start_loc(2), start_loc(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', 'r');
        hold off;

        xlabel('$x$ cell index', 'Interpreter', 'latex');
        ylabel('$y$ cell index', 'Interpreter', 'latex');
        title(['$t^{\mathrm{travel}}$, $v^{\mathrm{wind}} = ' num2str(v_w) '$ m/s'], 'Interpreter', 'latex');
        caxis([0 250]);  
    end

    sgtitle('Agent Travel Times for Different Wind Speeds');

    %% Additional Example Plots for Fire Spread Over Time (Wind Variation)
    try
        timesteps = [1, 21, 41];  
        wind_values = [0, 1, 3];  
        dt_e = 15;  
        start_fire = [11, 11];  

        figure_fire_spread = figure('Name', 'Fire Spread Over Time (Wind Variation)');
        set(gcf, 'Position', [100, 100, 900, 900]);

        for idx = 1:length(wind_values)
            config = i_sim_comms_disabled();
            environment_model = i_env_dynamics_20(config);
            structure_value = 0.5;  

            environment_model.v_w = wind_values(idx);
            environment_model.ang_w = pi / 4;
            environment_model.c_fs_1 = 0.1;
            environment_model.c_fs_2 = 1.2;
            environment_model.c_wm_1 = 0.15;
            environment_model.c_wm_2 = 1;
            environment_model.c_wm_d = 1;

            environment_model.m_s = ones(environment_model.n_x_e, environment_model.n_y_e) * structure_value;

            environment_model.m_f = ones(environment_model.n_x_e, environment_model.n_y_e);
            environment_model.m_f(start_fire(1), start_fire(2)) = 3; 
            environment_model.m_f_series(:, :, 1) = environment_model.m_f;

            for k_e = 1:max(timesteps)
                environment_model = model_environment(environment_model, k_e, dt_e);

                if ismember(k_e, timesteps)
                    plot_idx = (idx - 1) * length(timesteps) + find(timesteps == k_e);
                    subplot(length(wind_values), length(timesteps), plot_idx);

                    fire_map = environment_model.m_f_series(:, :, k_e);
                    imagesc(fire_map, [0 4]);
                    colormap(gca, myCmap);
                    caxis([0 4]);
                    colorbar; axis equal tight; set(gca, 'YDir', 'normal');
                    title(['$k = ' num2str(k_e) ', v^{\mathrm{wind}} = ' num2str(wind_values(idx)) 'm/s$'], 'Interpreter', 'latex');
                end
            end
        end
    catch
        warning('Skipping fire spread over time plot for wind variation due to missing dependencies.');
    end

    %% Additional Example Plots for Fire Spread Over Time (Structure Variation)
    try
        timesteps = [1, 21, 41];  
        structure_values = [0.25, 0.5, 1];  
        dt_e = 15;  
        start_fire = [11, 11];  
        v_w = 1; % Choose a representative wind speed or 0 if preferred
        ang_w = pi / 4; % Keep consistent wind angle

        figure_structure_spread = figure('Name', 'Fire Spread Over Time (Structure Variation)');
        set(gcf, 'Position', [100, 100, 900, 900]);

        for idx = 1:length(structure_values)
            config = i_sim_comms_disabled();
            environment_model = i_env_dynamics_20(config);
            
            % Set parameters similar to the wind variation example
            environment_model.v_w = 0;
            environment_model.ang_w = ang_w;
            environment_model.c_fs_1 = 0.1;
            environment_model.c_fs_2 = 1.2;
            environment_model.c_wm_1 = 0.15;
            environment_model.c_wm_2 = 1;
            environment_model.c_wm_d = 1;

            % Use the current structure value
            environment_model.m_s = ones(environment_model.n_x_e, environment_model.n_y_e) * structure_values(idx);

            % Set initial fire
            environment_model.m_f = ones(environment_model.n_x_e, environment_model.n_y_e);
            environment_model.m_f(start_fire(1), start_fire(2)) = 3; 
            environment_model.m_f_series(:, :, 1) = environment_model.m_f;

            for k_e = 1:max(timesteps)
                environment_model = model_environment(environment_model, k_e, dt_e);

                if ismember(k_e, timesteps)
                    plot_idx = (idx - 1) * length(timesteps) + find(timesteps == k_e);
                    subplot(length(structure_values), length(timesteps), plot_idx);

                    fire_map = environment_model.m_f_series(:, :, k_e);
                    imagesc(fire_map, [0 4]);
                    colormap(gca, myCmap);
                    caxis([0 4]);
                    colorbar; axis equal tight; set(gca, 'YDir', 'normal');
                    title(['$k = ' num2str(k_e) ', \mathbf{M}^{\mathrm{structure}} = ' num2str(structure_values(idx)) '$'], 'Interpreter', 'latex');
                end
            end
        end
    catch
        warning('Skipping fire spread over time plot for structure variation due to missing dependencies.');
    end

    %% FIS INPUT/OUTPUT PLOT PREPARATION (Assumes fisInputs and agent_model are in workspace)
    try
        figure; imagesc(fisInputs.cell_fire_time_risk); colorbar; axis equal tight; set(gca, 'YDir', 'normal');
        figure; imagesc(fisInputs.cell_scan_certainty); colorbar; axis equal tight; set(gca, 'YDir', 'normal');
        figure; imagesc(fisInputs.cell_priority); colorbar; axis equal tight; set(gca, 'YDir', 'normal');
        figure; imagesc(fisInputs.t_response); colorbar; axis equal tight; set(gca, 'YDir', 'normal');

        figure; imagesc(agent_model.m_bo_s); colorbar; axis equal tight; set(gca, 'YDir', 'normal');
        figure; imagesc(m_att); colorbar; axis equal tight; set(gca, 'YDir', 'normal');
    catch
        % If fisInputs or agent_model are not present, skip this section
        warning('Skipping FIS and agent model plots due to missing data.');
    end

end

% title(subplot(2,2,1), '$\mathbf{M}^{\mathrm{structure}}$', 'Interpreter', 'latex')
% title(subplot(2,2,2), '$\mathbf{M}^{\mathrm{building}}$', 'Interpreter', 'latex')
% title(subplot(2,2,3), '$\mathbf{M}^{\mathrm{c,building}}$', 'Interpreter', 'latex')
% title(subplot(2,2,4), '$\mathbf{M}^{\mathrm{victim}}$', 'Interpreter', 'latex')