function saveSimulationResults(flag, config)
    % Check if the flag is true
    if flag
        % Create a directory based on the current datetime within the specified save directory
        dateTimeFolder = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
        saveDir = fullfile(config.save_dir, dateTimeFolder);
        if ~exist(saveDir, 'dir')
            mkdir(saveDir);
        end
        
        % Save all workspace variables to a file within the new directory
        filename = fullfile(saveDir, 'simulationData.mat');
        evalin('caller', sprintf('save(''%s'')', filename)); % Correct formatting of the save command
        fprintf('All workspace variables saved to %s\n', filename);

        % Find all open figures
        figHandles = findobj('Type', 'figure');

        % Save each figure using its 'Name' property
        for i = 1:length(figHandles)
            try
                % Retrieve the figure's 'Name' property
                figName = get(figHandles(i), 'Name');
                % If the 'Name' property is empty, use a default name
                if isempty(figName)
                    figName = sprintf('Figure_%d', i);
                end
            catch
                % If any error occurs during retrieval, use a default name
                figName = sprintf('Figure_%d', i);
            end
            
            % Save the figure in FIG format
            figPathFig = fullfile(saveDir, [figName, '.fig']);
            savefig(figHandles(i), figPathFig);
            
            % Save the figure in PNG format
            figPathPng = fullfile(saveDir, [figName, '.png']);
            print(figHandles(i), figPathPng, '-dpng', '-r300'); % Save as PNG with 300 dpi resolution

            % Save the figure in SVG format
            figPathSvg = fullfile(saveDir, [figName, '.svg']);
            print(figHandles(i), figPathSvg, '-dsvg'); % Save as SVG

            fprintf('Figure "%s" saved as %s, %s, and %s\n', figName, figPathFig, figPathPng, figPathSvg);
        end
    else
        % Do not save, and output a message indicating this
        fprintf('Saving of simulation results is not enabled.\n');
    end
end
