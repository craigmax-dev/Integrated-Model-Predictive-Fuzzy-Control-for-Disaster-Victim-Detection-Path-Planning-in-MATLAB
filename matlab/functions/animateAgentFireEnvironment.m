function animateAgentFireEnvironment(agent_model, environment_model, config, output_filename)
    % Create a VideoWriter object to save the animation
    writerObj = VideoWriter(output_filename, 'MPEG-4');
    writerObj.FrameRate = 2; % Set frame rate for the animation
    open(writerObj);
    
    % Create a figure for the animation
    figure;
    colormap('hot'); % Set colormap for fire intensity

    % Define UAV markers and colors for different tasks
    uavColors = {'b', 'g', 'r', 'k'}; % Colors for different tasks
    uavMarkers = {'o', 's', 'd', '^'}; % Different shapes for different UAVs
    
    % Determine number of frames for the animation
    numFrames = min(size(agent_model.a_loc_hist, 3), size(environment_model.m_f_series, 3));

    k_e = 1;
    
    % Loop through each recorded timestep to generate animation frames
    for k = 1:numFrames

        % Coarsen the fire map for the current timestep
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
        for a = 1:size(agent_model.a_loc_hist, 1)
            uav_x = agent_model.a_loc_hist(a, 1, k);
            uav_y = agent_model.a_loc_hist(a, 2, k);
            uav_task = agent_model.a_task_hist(a, k);
            
            % Plot UAV location with color and marker corresponding to task
            if uav_task > 0 && uav_task <= length(uavColors)
                plot(uav_y, uav_x, uavMarkers{a}, 'MarkerSize', 12, 'MarkerEdgeColor', uavColors{uav_task}, 'MarkerFaceColor', uavColors{uav_task}, 'LineWidth', 1.5);
            end
        end
        hold off;

        % Capture the current frame and write it to the video file
        frame = getframe(gcf);
        writeVideo(writerObj, frame);

        %% Environment Model
        if k_e*config.dk_e <= k
          k_e = k_e + 1; 
        end

    end

    % Close the video file
    close(writerObj);
end
