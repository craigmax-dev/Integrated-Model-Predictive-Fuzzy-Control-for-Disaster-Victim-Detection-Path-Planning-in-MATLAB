% V2.2
function saveSimulationResults(flag, data, config)
    % Check if the flag is true
    if flag
        % Create a directory based on the current datetime within the specified save directory
        dateTimeFolder = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
        saveDir = fullfile(config.save_dir, dateTimeFolder);
        if ~exist(saveDir, 'dir')
            mkdir(saveDir);
        end
        
        % Save the data to a file within the new directory
        filename = fullfile(saveDir, 'simulationData.mat');
        save(filename, 'data');
        fprintf('Simulation results saved to %s\n', filename);

        % Find all open figures
        figHandles = findobj('Type', 'figure');

        % Save each figure using its title or a default name if the title is not available
        for i = 1:length(figHandles)
            try
                % Attempt to use the figure's title as its filename, default to Figure_N if unavailable
                axesHandle = get(figHandles(i), 'CurrentAxes');
                titleObject = get(axesHandle, 'Title');
                titleStr = get(titleObject, 'String');
                if isempty(titleStr)
                    figName = sprintf('Figure_%d', i);
                else
                    % Use only the first line if the title is multi-line
                    if iscell(titleStr)
                        titleStr = titleStr{1};
                    end
                    figName = matlab.lang.makeValidName(titleStr); % Ensure the title is a valid filename
                end
            catch
                % If any error occurs during title retrieval, use a default name
                figName = sprintf('Figure_%d', i);
            end
            
            figPathFig = fullfile(saveDir, [figName, '.fig']);
            figPathPng = fullfile(saveDir, [figName, '.png']);
            savefig(figHandles(i), figPathFig);
            print(figHandles(i), figPathPng, '-dpng'); % Save as PNG
            fprintf('Figure "%s" saved as %s and %s\n', figName, figPathFig, figPathPng);
        end
    else
        % Do not save, and output a message indicating this
        fprintf('Saving of simulation results is not enabled.\n');
    end
end


% % V2.1
% function saveSimulationResults(flag, data, config)
%     % Check if the flag is true
%     if flag
%         % Create a directory based on the current datetime within the specified save directory
%         dateTimeFolder = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
%         saveDir = fullfile(config.save_dir, dateTimeFolder);
%         if ~exist(saveDir, 'dir')
%             mkdir(saveDir);
%         end
% 
%         % Save the data to a file within the new directory
%         filename = fullfile(saveDir, 'simulationData.mat');
%         save(filename, 'data');
%         fprintf('Simulation results saved to %s\n', filename);
% 
%         % Find all open figures
%         figHandles = findobj('Type', 'figure');
% 
%         % Save each figure using its title
%         for i = 1:length(figHandles)
%             % Attempt to use the figure's title as its filename, default to figN if unavailable
%             figTitle = get(get(figHandles(i), 'CurrentAxes'), 'Title');
%             titleStr = get(figTitle, 'String');
%             if isempty(titleStr)
%                 figName = sprintf('Figure_%d', i);
%             else
%                 % Use only the first line if the title is multi-line
%                 if iscell(titleStr)
%                     titleStr = titleStr{1};
%                 end
%                 figName = matlab.lang.makeValidName(titleStr); % Ensure the title is a valid filename
%             end
% 
%             figPath = fullfile(saveDir, figName);
%             savefig(figHandles(i), [figPath, '.fig']);
%             print(figHandles(i), figPath, '-dpng'); % Save as PNG
%             fprintf('Figure "%s" saved as %s.fig and %s.png\n', figName, figPath, figPath);
%         end
%     else
%         % Do not save, and output a message indicating this
%         fprintf('Saving of simulation results is not enabled.\n');
%     end
% end
