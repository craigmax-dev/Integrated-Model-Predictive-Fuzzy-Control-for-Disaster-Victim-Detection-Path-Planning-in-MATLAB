% Plot timeseries parameter with stats

function plotStats(means, ci_lower, ci_upper, time_vector, simulationSetups, yLabel, lineStyles)
    % Check if line styles are provided, if not use default
    if nargin < 7 || isempty(lineStyles)
        lineStyles = repmat({'-', 'Color', [0, 0.4470, 0.7410]}, size(simulationSetups, 1), 1);
    end

    % Make the plot with a standard window size
    figure;
    set(gcf, 'Units', 'centimeters', 'Position', [1, 1, 25, 20]);
    hold on;

    % Increase text sizes for xlabel and ylabel
    xlabel('$t$ (s)', 'FontSize', 18, 'Interpreter', 'latex');
    ylabel(['$', yLabel, '$'], 'FontSize', 18, 'Interpreter', 'latex');
    
    meanHandles = [];
    meanLabels = {};
    
    for simSetup = 1:size(simulationSetups, 1)
        simulationName = simulationSetups{simSetup, 1}; % Assuming the simulation name is the first column
        lineSpec = lineStyles{simSetup}; % Get line specifications for each simulation
        
        % Plot means
        h = plot(time_vector{simSetup}, means{simSetup}, 'DisplayName', simulationName, 'LineStyle', lineSpec{1}, 'Color', lineSpec{2}, 'LineWidth', 2);
        
        % Shaded CI area
        X = [time_vector{simSetup}, fliplr(time_vector{simSetup})];
        Y = [ci_lower{simSetup}, fliplr(ci_upper{simSetup})];
        fill(X, Y, lineSpec{2}, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        
        % Adjusting yline for overall mean
        overall_mean = mean(means{simSetup}, 'omitnan');
        yline(overall_mean, '-', 'Color', lineSpec{2}, 'LabelHorizontalAlignment', 'left', 'LineWidth', 2, 'FontSize', 14, 'HandleVisibility', 'off');
        
        % Store the handle and label for the legend
        meanHandles = [meanHandles, h];
        meanLabels = [meanLabels, strcat(simulationName, " Mean: ", num2str(overall_mean, '%.2f'))];
    end
    
    % Create the legend for time series and mean lines
    legend(meanHandles, meanLabels, 'Location', 'northeast', 'FontSize', 14, 'Interpreter', 'latex');
    grid on;
    set(gca, 'FontSize', 14); % Increase axis labels and ticks font size
    
    % Adjust x-axis limits to data range
    xlim([min(cellfun(@(x) min(x), time_vector)), max(cellfun(@(x) max(x), time_vector))]);
end