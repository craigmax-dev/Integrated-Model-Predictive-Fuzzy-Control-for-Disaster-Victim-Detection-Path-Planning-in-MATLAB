function plotGeographical(agent_model, environment_model, agent_parameter_list, agent_labels, search_parameter_list, search_labels, items, item_locations, markerSizes, m_f_series_indexes)
    % Validate the input sizes
    if numel(agent_parameter_list) ~= numel(agent_labels) || numel(search_parameter_list) ~= numel(search_labels)
        error('The number of parameters and labels must match.');
    end

    % Determine if we only have one agent/environment parameter and one search parameter
    if numel(agent_parameter_list) == 1 && numel(search_parameter_list) == 1
        % Side by side layout
        nColumns = 2;
        nRows = 1;
    else
        % Original layout
        nColumns = max(numel(agent_parameter_list), numel(search_parameter_list));
        nRows = 2;
    end

    figure('Name', 'Environmental and Search Parameters with Items', 'NumberTitle', 'off');

    % Define symbols and colors for different items
    itemSymbols = {'o', '^', 's', 'd'}; 
    itemColors = {[1 0 0], [0 0 1], [0 1 0], [0.75 0.75 0]};

    % Scaling factors for translating search map coordinates to environment map coordinates
    scaleX = size(environment_model.m_f, 2) / agent_model.n_x_s;
    scaleY = size(environment_model.m_f, 1) / agent_model.n_y_s;

    % Plot agent/environment parameters
    for idx = 1:numel(agent_parameter_list)
        subplot(nRows, nColumns, idx);
        parameterName = agent_parameter_list{idx};
        parameterValue = environment_model.(parameterName);
        imagesc(parameterValue); 
        hold on;

        % Plot items on environment map with translated coordinates
        for itemIdx = 1:length(items)
            locations = item_locations{itemIdx};
            % Translate coordinates
            translatedLocations = [locations(:,1) * scaleX, locations(:,2) * scaleY];
            for locIdx = 1:size(translatedLocations, 1)
                plot(translatedLocations(locIdx, 1), translatedLocations(locIdx, 2), ...
                    itemSymbols{mod(itemIdx-1, length(itemSymbols))+1}, ...
                    'MarkerEdgeColor', 'k', ...
                    'MarkerFaceColor', itemColors{mod(itemIdx-1, length(itemColors))+1}, ...
                    'MarkerSize', markerSizes(itemIdx));
            end
        end

        hold off;
        colorbar;
        axis equal tight;
        set(gca, 'YDir', 'normal'); 
        title(['$', agent_labels{idx}, '$'], 'Interpreter', 'latex');
        xlabel('$x$ cell index', 'Interpreter', 'latex');
        ylabel('$y$ cell index', 'Interpreter', 'latex');
    end

    % Plot search parameters
    for idx = 1:numel(search_parameter_list)
        % If single param each, place search on the second subplot in one row
        if numel(agent_parameter_list) == 1 && numel(search_parameter_list) == 1
            subplot(nRows, nColumns, 2); 
        else
            subplot(nRows, nColumns, nColumns + idx); 
        end
        parameterName = search_parameter_list{idx};
        parameterValue = agent_model.(parameterName);

        imagesc(parameterValue);
        hold on;
        colorbar;

        % Plot items on search map (no coordinate translation)
        for itemIdx = 1:length(items)
            locations = item_locations{itemIdx};
            for locIdx = 1:size(locations, 1)
                plot(locations(locIdx, 1), locations(locIdx, 2), ...
                    itemSymbols{mod(itemIdx-1, length(itemSymbols))+1}, ...
                    'MarkerEdgeColor', 'k', ...
                    'MarkerFaceColor', itemColors{mod(itemIdx-1, length(itemColors))+1}, ...
                    'MarkerSize', markerSizes(itemIdx));
            end
        end

        hold off;
        axis equal tight;
        set(gca, 'YDir', 'normal');
        title(['$', search_labels{idx}, '$'], 'Interpreter', 'latex');
        xlabel('$x$ cell index', 'Interpreter', 'latex');
        ylabel('$y$ cell index', 'Interpreter', 'latex');
    end

    % Plot m_f_series for fire map evolution
    figure('Name', 'Fire Map Evolution', 'NumberTitle', 'off', ...
           'Units', 'normalized', 'OuterPosition', [0 0 1 1]);

    numSeries = length(m_f_series_indexes);

    myCmap = [
        0.8,   0.8,   0.8;    % 0: non-flammable
        0.9,   0.95,  1.0;    % 1: flammable
        1.0,   0.498, 0.0;    % 2: catching fire
        0.894, 0.102, 0.110;  % 3: burning
        0.0,   0.0,   0.0     % 4: extinguished
    ];

    for idx = 1:numSeries
        subplot(1, numSeries + 1, idx); 
        imagesc(environment_model.m_f_series(:,:,m_f_series_indexes(idx)), [0 4]);
        hold on;
        colormap(gca, myCmap);
        caxis([0 4]);
        hold off;
        axis equal tight;
        set(gca, 'YDir', 'normal'); 
        title(['$t = ', num2str(60*m_f_series_indexes(idx)), 's$'], 'Interpreter', 'latex');
        xlabel('$x$ cell index', 'Interpreter', 'latex');
        ylabel('$y$ cell index', 'Interpreter', 'latex');
    end

    subplotHandle = subplot(1, numSeries + 1, numSeries + 1);
    pos = get(subplotHandle, 'Position'); 
    delete(subplotHandle); 

    cb = colorbar('Position', [pos(1) + 0.02, pos(2), 0.02, pos(4)]); 
    cb.Ticks = 0:1:4; 
    cb.TickLabels = {'Non-flammable','Flammable','Catching Fire','Burning','Extinguished'};
end
