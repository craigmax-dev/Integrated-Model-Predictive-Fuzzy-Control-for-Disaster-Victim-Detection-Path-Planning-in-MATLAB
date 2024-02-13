% Plot environment and search variables

% TODO
% - Add wrapper function to build animations
% - Add legend for shapes


% % V2.3
function plotGeographical(agent_model, environment_model, agent_parameter_list, search_parameter_list, items, item_locations, markerSizes)
    % Determine the number of plots
    totalPlots = numel(agent_parameter_list) + numel(search_parameter_list);
    nColumns = max(numel(agent_parameter_list), numel(search_parameter_list)); % Ensure horizontal alignment
    nRows = 2; % One row for agent/environment parameters, one for search parameters

    figure('Name', 'Environmental and Search Parameters with Items', 'NumberTitle', 'off');

    % Define symbols and colors for different items
    itemSymbols = {'o', '^', 's', 'd'}; % E.g., circle for UAV, triangle for charging station
    itemColors = {[1 0 0], [0 0 1], [0 1 0], [0.75 0.75 0]}; % E.g., red for UAV, blue for charging station

    % Scaling factors for translating search map coordinates to environment map coordinates
    scaleX = size(environment_model.m_f, 2) / agent_model.n_x_s;
    scaleY = size(environment_model.m_f, 1) / agent_model.n_y_s;

    % Plot agent/environment parameters
    for idx = 1:numel(agent_parameter_list)
        subplot(2, nColumns, idx);
        parameterName = agent_parameter_list{idx};
        parameterValue = environment_model.(parameterName);
        imagesc(parameterValue); % Use imagesc for automatic scaling
        hold on;

        % Plot items on environment map with translated coordinates
        for itemIdx = 1:length(items)
            locations = item_locations{itemIdx};
            % Translate coordinates
            translatedLocations = [locations(:,1) * scaleX, locations(:,2) * scaleY];
            for locIdx = 1:size(translatedLocations, 1)
                plot(translatedLocations(locIdx, 1), translatedLocations(locIdx, 2), itemSymbols{mod(itemIdx-1, length(itemSymbols))+1}, ...
                    'MarkerEdgeColor', 'k', 'MarkerFaceColor', itemColors{mod(itemIdx-1, length(itemColors))+1}, ...
                    'MarkerSize', markerSizes(itemIdx));
            end
        end

        hold off;
        colorbar;
        axis equal tight;
        title(strrep(parameterName, '_', ' '));
        xlabel('X (m)');
        ylabel('Y (m)');
    end

    % Plot search parameters
    for idx = 1:numel(search_parameter_list)
        subplot(2, nColumns, nColumns + idx); % Adjusted for vertical alignment
        parameterName = search_parameter_list{idx};
        parameterValue = agent_model.(parameterName);

        imagesc(parameterValue);
        hold on;
        colorbar;

        % Plot items on search map without translating coordinates
        for itemIdx = 1:length(items)
            locations = item_locations{itemIdx};
            for locIdx = 1:size(locations, 1)
                plot(locations(locIdx, 1), locations(locIdx, 2), itemSymbols{mod(itemIdx-1, length(itemSymbols))+1}, ...
                    'MarkerEdgeColor', 'k', 'MarkerFaceColor', itemColors{mod(itemIdx-1, length(itemColors))+1}, ...
                    'MarkerSize', markerSizes(itemIdx));
            end
        end

        hold off;
        axis equal tight;
        title(strrep(parameterName, '_', ' '));
        xlabel('X (m)');
        ylabel('Y (m)');
    end
end



% % V2.2
% function plotGeographical(agent_model, environment_model, agent_parameter_list, search_parameter_list, items, item_locations, markerSizes)
%     % Determine the number of plots
%     totalPlots = numel(agent_parameter_list) + numel(search_parameter_list);
%     nColumns = ceil(sqrt(totalPlots)); % Arrange plots in a square-ish layout
%     nRows = ceil(totalPlots / nColumns);
% 
%     figure('Name', 'Environmental and Search Parameters with Items', 'NumberTitle', 'off');
% 
%     % Define symbols and colors for different items
%     itemSymbols = {'o', '^', 's', 'd'}; % E.g., circle for UAV, triangle for charging station
%     itemColors = {[1 0 0], [0 0 1], [0 1 0], [0.75 0.75 0]}; % E.g., red for UAV, blue for charging station
% 
%     % Initialize a variable to keep track of legend entries
%     legendEntries = {};
% 
%     % Plot agent/environment parameters
%     for idx = 1:numel(agent_parameter_list)
%         subplot(nRows, nColumns, idx);
%         parameterName = agent_parameter_list{idx};
%         parameterValue = environment_model.(parameterName);
%         imagesc(parameterValue); % Use imagesc for automatic scaling
%         hold on;
% 
%         % Special handling for fire map
%         if strcmp(parameterName, 'm_f')
%             colormap(gca, [1 1 1; 0 1 0; 1 0.5 0; 1 0 0; 0 0 1]); % Custom colormap for fire states
%             caxis([0 4]); % Limit color axis for fire states
%         end
% 
%         % Plot items
%         for itemIdx = 1:length(items)
%             locations = item_locations{itemIdx};
%             for locIdx = 1:size(locations, 1)
%                 plot(locations(locIdx, 1), locations(locIdx, 2), itemSymbols{mod(itemIdx-1, length(itemSymbols))+1}, ...
%                     'MarkerEdgeColor', 'k', 'MarkerFaceColor', itemColors{mod(itemIdx-1, length(itemColors))+1}, ...
%                     'MarkerSize', markerSizes(itemIdx));
%             end
%             legendEntries{end+1} = items{itemIdx}; % Add item to legend entries
%         end
% 
%         hold off;
%         colorbar; % Add color scale
%         axis equal tight;
%         set(gca, 'XLim', [1 size(parameterValue, 2)], 'YLim', [1 size(parameterValue, 1)]); % Fix dimensions
%         title(strrep(parameterName, '_', ' '));
%         xlabel('X (m)');
%         ylabel('Y (m)');
%     end
% 
%     % Plot search parameters
%     for idx = 1:numel(search_parameter_list)
%         subplot(nRows, nColumns, numel(agent_parameter_list) + idx);
%         parameterName = search_parameter_list{idx};
%         parameterValue = agent_model.(parameterName);
% 
%         % Adjust search parameter grid to match environmental grid resolution
%         adjustedValue = imresize(parameterValue, [size(environment_model.m_f, 1), size(environment_model.m_f, 2)], 'nearest');
% 
%         imagesc(adjustedValue);
%         hold on;
%         colorbar;
% 
%         % Plot items
%         for itemIdx = 1:length(items)
%             locations = item_locations{itemIdx};
%             for locIdx = 1:size(locations, 1)
%                 plot(locations(locIdx, 1), locations(locIdx, 2), itemSymbols{mod(itemIdx-1, length(itemSymbols))+1}, ...
%                     'MarkerEdgeColor', 'k', 'MarkerFaceColor', itemColors{mod(itemIdx-1, length(itemColors))+1}, ...
%                     'MarkerSize', markerSizes(itemIdx));
%             end
%         end
% 
%         hold off;
%         axis equal tight;
%         set(gca, 'XLim', [1 size(adjustedValue, 2)], 'YLim', [1 size(adjustedValue, 1)]); % Fix dimensions
%         title(strrep(parameterName, '_', ' '));
%         xlabel('X (m)');
%         ylabel('Y (m)');
%     end
% 
%     % Add legend for items
%     legend(legendEntries, 'Location', 'bestoutside');
% end



