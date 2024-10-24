function animateFLCParams(flc_params_hist, fisArray, outputFileName)
    % Validate inputs
    if nargin < 3
        error('Please provide flc_params_hist, fisArray, and the output file name.');
    end

    % Number of time steps is the number of columns in flc_params_hist
    numTimeSteps = size(flc_params_hist, 2);
    % Number of agents is the number of rows in flc_params_hist
    numAgents = size(flc_params_hist, 1);

    % Determine the number of outputs and membership functions (assuming all FIS have the same structure)
    numOutputs = numel(fisArray(1).Outputs);
    numMFs = numel(fisArray(1).Outputs(1).MembershipFunctions);

    % Loop over each agent to create separate animations
    for agentIdx = 1:numAgents
        % Create figure for animation
        figure;
        sgtitle(sprintf('FLC Output MF Parameters Evolution for Agent %d', agentIdx), 'Interpreter', 'latex');
        colors = lines(1);  % Use a distinct color for the agent

        % Create video writer object to export animation as MP4
        videoWriter = VideoWriter(sprintf('%s_agent_%d.mp4', outputFileName, agentIdx), 'MPEG-4');
        open(videoWriter);

        % Loop over each time step
        for t = 1:numTimeSteps
            clf; % Clear the current figure

            % Plot each membership function at this time step
            for mfIndex = 1:numMFs
                subplot(numMFs, 1, mfIndex);
                hold on;

                % Plot the FIS parameters for this MF
                fisParams = flc_params_hist{agentIdx, t};
                if iscell(fisParams) && ~isempty(fisParams)
                    paramsMatrix = fisParams{1}{mfIndex};

                    % Prepare labels for parameters using the input names and LaTeX
                    labels = arrayfun(@(n) strcat('$', strrep(fisArray(1).Inputs(n).Name, '_', '\_'), '$'), 1:length(paramsMatrix) - 1, 'UniformOutput', false);
                    labels{end + 1} = '$c$';  % Label for constant term

                    % Convert labels to cell array of character vectors
                    labels = cellstr(labels);  % This should ensure the correct format for parallelcoords

                    % Plot parameters with distinct color for the agent
                    parallelcoords(paramsMatrix, 'Labels', labels, 'Color', colors(1,:), 'Marker', 'o', 'DisplayName', sprintf('Agent %d', agentIdx));
                end

                % Customize subplot for this MF
                title(sprintf('MF "%s" at time step %d', fisArray(1).Outputs(1).MembershipFunctions(mfIndex).Name, t), 'Interpreter', 'latex');
                xlabel('Parameter Index', 'Interpreter', 'latex');
                ylabel('Value', 'Interpreter', 'latex');
                grid on;
                legend show;

                hold off;
            end

            % Capture current frame for video
            frame = getframe(gcf);
            writeVideo(videoWriter, frame);
        end

        % Close video writer
        close(videoWriter);

        % Close the figure
        close;
    end
end
