function plotStats(means, ci_lower, ci_upper, time_vector, simulationSetups, yLabel, lineStyles)

    % Check if line styles are provided, if not use default
    if nargin < 7 || isempty(lineStyles)
        lineStyles = repmat({{'-', [0, 0.4470, 0.7410]}}, size(simulationSetups, 1), 1);
    end

    % Function to convert hex to RGB
    hex2rgb = @(hex) sscanf(hex(2:end), '%2x%2x%2x', [1 3]) / 255;

    % Make the plot with a standard window size
    figHandle = figure;
    set(figHandle, 'Units', 'centimeters', 'Position', [1, 1, 25, 20]);
    hold on;

    % Increase text sizes for xlabel and ylabel
    xlabel('Simulation Time, $t$ (s)', 'FontSize', 18, 'Interpreter', 'latex');
    ylabel(yLabel, 'FontSize', 18, 'Interpreter', 'latex');
    
    % Set the figure 'Name' property based on yLabel
    % Map yLabel to a simplified label for the figure 'Name' property
    switch yLabel
        case "Objective Function, $\overline{J}$"
            set(figHandle, 'Name', 'obj');
        case "Optimisation Time, $\overline{t}^{\mathrm{opt}}$ (s)"
            set(figHandle, 'Name', 't_opt');
        otherwise
            % Set a default name if yLabel does not match known labels
            set(figHandle, 'Name', 'Figure');
    end

    meanHandles = [];
    meanLabels = {};
    
    for simSetup = 1:size(simulationSetups, 1)
        % Skip if the means data is empty
        if isempty(means{simSetup})
            continue;
        end

        simulationName = simulationSetups{simSetup, 7}; % Assuming the simulation name is the seventh column
        lineSpec = lineStyles{simSetup}; % Get line specifications for each simulation
        
        % Extract the line style and color from lineSpec
        lineStyle = lineSpec{1};
        colorHex = lineSpec{2};
        
        % Convert hex color to RGB
        color = hex2rgb(colorHex);
        
        % Plot means with the equivalent line style
        h = plot(time_vector{simSetup}, means{simSetup}, 'DisplayName', simulationName, 'LineStyle', lineStyle, 'Color', color, 'LineWidth', 2);

        % Shaded CI area
        X = [time_vector{simSetup}, fliplr(time_vector{simSetup})];
        Y = [ci_lower{simSetup}, fliplr(ci_upper{simSetup})];
        fill(X, Y, color, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        
        % Adjusting yline for overall mean with the equivalent line style
        overall_mean = mean(means{simSetup}, 'omitnan');
        yline(overall_mean, lineStyle, 'Color', color, 'LabelHorizontalAlignment', 'left', 'LineWidth', 2, 'FontSize', 14, 'HandleVisibility', 'off');
        
        % Store the handle and label for the legend
        meanHandles = [meanHandles, h];
        meanLabels = [meanLabels, strcat(simulationName, " Mean: ", num2str(overall_mean, '%.2f'))];
    end
    
    % Create the legend for time series and mean lines
    if ~isempty(meanHandles)
        legend(meanHandles, meanLabels, 'Location', 'northeast', 'FontSize', 14, 'Interpreter', 'latex');
    end
    grid on;
    set(gca, 'FontSize', 14); % Increase axis labels and ticks font size
    
    % Adjust x-axis limits to data range, excluding empty time vectors
    % Collect min and max values from non-empty time vectors
    min_values = [];
    max_values = [];
    for idx = 1:length(time_vector)
        if ~isempty(time_vector{idx})
            min_values = [min_values, min(time_vector{idx})];
            max_values = [max_values, max(time_vector{idx})];
        end
    end

    % Set x-axis limits if there are valid min and max values
    if ~isempty(min_values) && ~isempty(max_values)
        xlim([min(min_values), max(max_values)]);
    end
end
