function animateCombinedFLCAndAgentFire(agent_model, environment_model, config, flc_params_hist, fisArray, means_obj, outputFileName)
    % Validate inputs
    if nargin < 7
        error('Please provide agent_model, environment_model, config, flc_params_hist, fisArray, means_obj, and the output file name.');
    end

    % Number of time steps is the length of agent_model.a_task_hist
    numTimeSteps = size(agent_model.a_task_hist, 2);
    % Number of agents is the number of rows in flc_params_hist
    numAgents = size(flc_params_hist, 1);

    % Determine the number of outputs and membership functions (assuming all FIS have the same structure)
    numOutputs = numel(fisArray(1).Outputs);
    numMFs = numel(fisArray(1).Outputs(1).MembershipFunctions);

    % Create a VideoWriter object to save the combined animation
    writerObj = VideoWriter(outputFileName, 'MPEG-4');
    writerObj.FrameRate = 2; % Set frame rate for the animation
    open(writerObj);

    % Create figure for the animation
    figure('Position', [100, 100, 1800, 900]); % Wider figure with extra space for objective plot

    % Define UAV markers and colors for different tasks
    uavColors = {'b', 'g', 'r', 'k'}; % Colors for different tasks
    uavMarkers = {'o', 's', 'd', '^'}; % Different shapes for different UAVs

    % Loop through each time step to create the animation frames
    for k = 1:numTimeSteps
        clf; % Clear the current figure

        % Plot the fire map and agent locations (Top left subplot)
        subplot(3, 2, [1, 3]);

        % Determine the corresponding environment time step
        k_e = ceil(k / config.dk_e); 
        if k_e > size(environment_model.m_f_series, 3)
            break;
        end
        m_f = environment_model.m_f_series(:, :, k_e);
        [m_f_coarsened, ~] = func_coarsen(m_f, config.c_f_s);

        % Plot the coarsened fire map
        imagesc(m_f_coarsened, [0 4]); % Set scale to range 0-4
        colorbar;
        axis equal tight;
        set(gca, 'YDir', 'normal');
        xlabel('$x$ cell index', 'Interpreter', 'latex');
        ylabel('$y$ cell index', 'Interpreter', 'latex');
        title(['Fire Map with UAVs at Timestep ', num2str(k)], 'Interpreter', 'latex');
        hold on;

        % Overlay the UAVs at the current timestep
        for a = 1:numAgents
            uav_x = agent_model.a_loc_hist(a, 1, k);
            uav_y = agent_model.a_loc_hist(a, 2, k);
            uav_task = agent_model.a_task_hist(a, k);

            % Plot UAV location with color and marker corresponding to task
            if uav_task > 0 && uav_task <= length(uavColors)
                plot(uav_y, uav_x, uavMarkers{a}, 'MarkerSize', 12, 'MarkerEdgeColor', uavColors{uav_task}, 'MarkerFaceColor', uavColors{uav_task}, 'LineWidth', 1.5);
            end
        end
        hold off;

        % Plot the FLC parameters for all agents (Top right subplots)
        k_mpc = ceil(k / config.dk_mpc);
        sgtitle('FLC Output MF Parameters Evolution', 'Interpreter', 'latex');
        colors = lines(numAgents);  % Use distinct colors for each agent

        % Plot each membership function for all agents
        for mfIndex = 1:numMFs
            subplot(3, 2, mfIndex * 2);
            hold on;
            for agentIdx = 1:numAgents
                fisParams = flc_params_hist{agentIdx, k_mpc};
                if iscell(fisParams) && ~isempty(fisParams)
                    paramsMatrix = fisParams{1}{mfIndex};

                    % Plot parameters with distinct color for each agent
                    plot(1:length(paramsMatrix), paramsMatrix, '-o', 'Color', colors(agentIdx, :), 'DisplayName', sprintf('Agent %d', agentIdx));
                end
            end

            % Customize subplot for the FLC parameters
            title(sprintf('MF "%s"', fisArray(1).Outputs(1).MembershipFunctions(mfIndex).Name), 'Interpreter', 'latex');
            xlabel('Parameter Index', 'Interpreter', 'latex');
            ylabel('Value', 'Interpreter', 'latex');
            grid on;
            legend show;
            hold off;
        end

        % Plot the objective function history (Bottom subplot)
        subplot(3, 2, [5, 6]);
        plot(1:length(means_obj), means_obj, 'b-', 'LineWidth', 2);
        hold on;
        % Plot a line indicating the current timestep
        xline(k, 'r--', 'LineWidth', 2, 'DisplayName', 'Current Timestep');
        xlabel('Time Step', 'Interpreter', 'latex');
        ylabel('Objective Value', 'Interpreter', 'latex');
        title('Objective Function History', 'Interpreter', 'latex');
        grid on;
        hold off;

        % Capture the current frame and write it to the video file
        frame = getframe(gcf);
        writeVideo(writerObj, frame);
    end

    % Close the video file
    close(writerObj);
    % Close the figure
    close;
end
