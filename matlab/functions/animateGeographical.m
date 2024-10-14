% TODO
% - validate

function animateGeographical(agent_models, environment_model, agent_parameter_list, search_parameter_list, items, item_locations_over_time, markerSizes, fileName)
    % Create a video writer object with 1 frame per second
    v = VideoWriter(fileName, 'MPEG-4');
    v.FrameRate = 1; % Adjust frame rate as needed
    open(v);
    
    % Number of timesteps
    numTimesteps = length(agent_models);
    
    % Create a figure for the animation
    fig = figure('Name', 'Parameter Animation', 'NumberTitle', 'off');
    
    for timestep = 1:numTimesteps
        % Update agent model for current timestep
        agent_model = agent_models{timestep};
        
        % Update item locations for current timestep
        item_locations = item_locations_over_time{timestep};
        
        % Call the plotting function for the current timestep
        % Note: You might need to adjust this function call according to your specific implementation
        plotGeographical(agent_model, environment_model, agent_parameter_list, search_parameter_list, items, item_locations, markerSizes);
        
        % Capture the current figure/frame
        frame = getframe(fig);
        writeVideo(v, frame);
        
        % Clear the figure to prepare for the next frame
        clf(fig);
    end
    
    % Close the video file
    close(v);
    close(fig); % Close the figure window
end
