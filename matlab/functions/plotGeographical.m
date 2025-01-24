% V2
function plotGeographical(agent_model, environment_model, agent_parameter_list, agent_labels, search_parameter_list, search_labels, items, item_locations, markerSizes, m_f_series_indexes)
    % Validate the input sizes
    if numel(agent_parameter_list) ~= numel(agent_labels) || numel(search_parameter_list) ~= numel(search_labels)
        error('The number of parameters and labels must match.');
    end

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

    % Plot agent/environment parameters with custom LaTeX titles
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
        set(gca, 'YDir', 'normal'); % Reverse the y-axis
        title(['$', agent_labels{idx}, '$'], 'Interpreter', 'latex'); % Use LaTeX for the title
        xlabel('$x$ cell index', 'Interpreter', 'latex');
        ylabel('$y$ cell index', 'Interpreter', 'latex');
    end

    % Plot search parameters with custom LaTeX titles
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
        set(gca, 'YDir', 'normal'); % Reverse the y-axis
        title(['$', search_labels{idx}, '$'], 'Interpreter', 'latex'); % Use LaTeX for the title
        xlabel('$x$ cell index', 'Interpreter', 'latex');
        ylabel('$y$ cell index', 'Interpreter', 'latex');
    end

    % Plot m_f_series as horizontally arranged subplots and set to fullscreen
    figure('Name', 'Fire Map Evolution', 'NumberTitle', 'off', 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);

    numSeries = length(m_f_series_indexes);

    % Define a custom discrete colormap for the fire map
    myCmap = [
        0.8,   0.8,   0.8;    % 0: non-flammable (light gray)
        0.9,   0.95,  1.0;    % 1: flammable (pale blue)
        1.0,   0.498, 0.0;    % 2: catching fire (orange)
        0.894, 0.102, 0.110;  % 3: burning (red)
        0.0,   0.0,   0.0     % 4: extinguished (black)
    ];

    for idx = 1:numSeries
        subplot(1, numSeries + 1, idx); % Arrange subplots horizontally
        imagesc(environment_model.m_f_series(:,:,m_f_series_indexes(idx)), [0 4]); % Plot fire map at each index
        hold on;
        colormap(gca, myCmap); % Apply the custom colormap to the current axes
        caxis([0 4]); % Ensure colormap covers the range of fire states
        hold off;
        axis equal tight;
        set(gca, 'YDir', 'normal'); % Reverse the y-axis
        title(['$t = ', num2str(60*m_f_series_indexes(idx)), 's$'], 'Interpreter', 'latex');
        xlabel('$x$ cell index', 'Interpreter', 'latex');
        ylabel('$y$ cell index', 'Interpreter', 'latex');
    end

    % Add a single shared colorbar/legend to the right of the last subplot
    subplotHandle = subplot(1, numSeries + 1, numSeries + 1); % Add an extra subplot for the legend
    pos = get(subplotHandle, 'Position'); % Get the position of the last subplot
    delete(subplotHandle); % Remove the unnecessary subplot

    cb = colorbar('Position', [pos(1) + 0.02, pos(2), 0.02, pos(4)]); % Adjust colorbar position
    cb.Ticks = 0:1:4; % Show ticks at each integer value
    cb.TickLabels = {'Non-flammable', 'Flammable', 'Catching Fire', 'Burning', 'Extinguished'};
end


% V1
% function plotGeographical(agent_model, environment_model, agent_parameter_list, agent_labels, search_parameter_list, search_labels, items, item_locations, markerSizes, m_f_series_indexes)
%     % Validate the input sizes
%     if numel(agent_parameter_list) ~= numel(agent_labels) || numel(search_parameter_list) ~= numel(search_labels)
%         error('The number of parameters and labels must match.');
%     end
% 
%     % Determine the number of plots
%     totalPlots = numel(agent_parameter_list) + numel(search_parameter_list);
%     nColumns = max(numel(agent_parameter_list), numel(search_parameter_list)); % Ensure horizontal alignment
%     nRows = 2; % One row for agent/environment parameters, one for search parameters
% 
%     figure('Name', 'Environmental and Search Parameters with Items', 'NumberTitle', 'off');
% 
%     % Define symbols and colors for different items
%     itemSymbols = {'o', '^', 's', 'd'}; % E.g., circle for UAV, triangle for charging station
%     itemColors = {[1 0 0], [0 0 1], [0 1 0], [0.75 0.75 0]}; % E.g., red for UAV, blue for charging station
% 
%     % Scaling factors for translating search map coordinates to environment map coordinates
%     scaleX = size(environment_model.m_f, 2) / agent_model.n_x_s;
%     scaleY = size(environment_model.m_f, 1) / agent_model.n_y_s;
% 
%     % Plot agent/environment parameters with custom LaTeX titles
%     for idx = 1:numel(agent_parameter_list)
%         subplot(2, nColumns, idx);
%         parameterName = agent_parameter_list{idx};
%         parameterValue = environment_model.(parameterName);
%         imagesc(parameterValue); % Use imagesc for automatic scaling
%         hold on;
% 
%         % Plot items on environment map with translated coordinates
%         for itemIdx = 1:length(items)
%             locations = item_locations{itemIdx};
%             % Translate coordinates
%             translatedLocations = [locations(:,1) * scaleX, locations(:,2) * scaleY];
%             for locIdx = 1:size(translatedLocations, 1)
%                 plot(translatedLocations(locIdx, 1), translatedLocations(locIdx, 2), itemSymbols{mod(itemIdx-1, length(itemSymbols))+1}, ...
%                     'MarkerEdgeColor', 'k', 'MarkerFaceColor', itemColors{mod(itemIdx-1, length(itemColors))+1}, ...
%                     'MarkerSize', markerSizes(itemIdx));
%             end
%         end
% 
%         hold off;
%         colorbar;
%         axis equal tight;
%         set(gca, 'YDir', 'normal'); % Reverse the y-axis
%         title(['$', agent_labels{idx}, '$'], 'Interpreter', 'latex'); % Use LaTeX for the title
%         xlabel('$x$ cell index', 'Interpreter', 'latex');
%         ylabel('$y$ cell index', 'Interpreter', 'latex');
%     end
% 
%     % Plot search parameters with custom LaTeX titles
%     for idx = 1:numel(search_parameter_list)
%         subplot(2, nColumns, nColumns + idx); % Adjusted for vertical alignment
%         parameterName = search_parameter_list{idx};
%         parameterValue = agent_model.(parameterName);
% 
%         imagesc(parameterValue);
%         hold on;
%         colorbar;
% 
%         % Plot items on search map without translating coordinates
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
%         set(gca, 'YDir', 'normal'); % Reverse the y-axis
%         title(['$', search_labels{idx}, '$'], 'Interpreter', 'latex'); % Use LaTeX for the title
%         xlabel('$x$ cell index', 'Interpreter', 'latex');
%         ylabel('$y$ cell index', 'Interpreter', 'latex');
%     end
% 
%     % Plot m_f_series as horizontally arranged subplots and set to fullscreen
%     figure('Name', 'Fire Map Evolution', 'NumberTitle', 'off', 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
% 
%     numSeries = length(m_f_series_indexes);
% 
%     for idx = 1:numSeries
%         subplot(1, numSeries + 1, idx); % Arrange subplots horizontally
%         imagesc(environment_model.m_f_series(:,:,m_f_series_indexes(idx))); % Plot fire map at each index
%         hold on;
% 
%         hold off;
%         axis equal tight;
%         set(gca, 'YDir', 'normal'); % Reverse the y-axis
%         title(['$k = ', num2str(m_f_series_indexes(idx)), '$'], 'Interpreter', 'latex');
%         xlabel('$x$ cell index', 'Interpreter', 'latex');
%         ylabel('$y$ cell index', 'Interpreter', 'latex');
%     end
% 
%     % Add a single shared colorbar/legend to the right of the last subplot
%     subplotHandle = subplot(1, numSeries + 1, numSeries + 1); % Add an extra subplot for the legend
%     pos = get(subplotHandle, 'Position'); % Get the position of the last subplot
%     delete(subplotHandle); % Remove the unnecessary subplot
% 
%     colorbar('Position', [pos(1) + 0.02, pos(2), 0.02, pos(4)]); % Adjust colorbar position and make it the same height
% end