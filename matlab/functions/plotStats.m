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


% % V3
% function plotStats(means, ci_lower, ci_upper, time_vector, simulationSetups, yLabel, battery)
%     % Make the plot with a standard window size
%     figure;
%     set(gcf, 'Units', 'centimeters', 'Position', [1, 1, 25, 20]);
%     hold on;
% 
%     % Increase text sizes for xlabel and ylabel
%     xlabel('$t$ (s)', 'FontSize', 18, 'Interpreter', 'latex');
%     ylabel('$\overline{J}$', 'FontSize', 18, 'Interpreter', 'latex');
% 
%     % Predefined colors for consistency and clarity
%     colors = [0, 0.4470, 0.7410; 0.8500, 0.3250, 0.0980; 0.9290, 0.6940, 0.1250; 0.4940, 0.1840, 0.5560; 0.4660, 0.6740, 0.1880; 0.3010, 0.7450, 0.9330; 0.6350, 0.0780, 0.1840];
%     colorIndex = 1;
% 
%     meanHandles = [];
%     meanLabels = {};
% 
%     for simSetup = 1:size(simulationSetups, 1)
%         simulationName = simulationSetups{simSetup, 7};
%         color = colors(mod(colorIndex - 1, size(colors, 1)) + 1, :); % Cycle through predefined colors
%         colorIndex = colorIndex + 1;
% 
%         % Plot means
%         h = plot(time_vector{simSetup}, means{simSetup}, 'DisplayName', simulationName, 'LineStyle', '-', 'Color', color, 'LineWidth', 2);
% 
%         % Shaded CI area
%         X = [time_vector{simSetup}, fliplr(time_vector{simSetup})];
%         Y = [ci_lower{simSetup}, fliplr(ci_upper{simSetup})];
%         fill(X, Y, color, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
% 
%         % Adjusting yline for overall mean with increased 'FontSize'
%         overall_mean = mean(means{simSetup}, 'omitnan');
%         yMean = yline(overall_mean, '-', 'Color', color, 'LabelHorizontalAlignment', 'left', 'LineWidth', 2, 'FontSize', 14, 'HandleVisibility', 'off');
% 
%         % Store the handle and label for the legend
%         meanHandles = [meanHandles, h];
%         meanLabels = [meanLabels, strcat(simulationName, " Mean: ", num2str(overall_mean))];
% 
%         % Optionally plot the means for each segment defined in the battery vector
%         if nargin > 7 && ~isempty(battery)
%             for i = 1:length(battery)
%                 segment_indices = time_vector{simSetup} <= battery(i);
%                 segment_mean = mean(means{simSetup}(segment_indices), 'omitnan');
%                 yline(segment_mean, ':', 'Color', color, 'LabelHorizontalAlignment', 'left', 'LineWidth', 1.2, 'FontSize', 14, 'HandleVisibility', 'off');
%             end
%         end
%     end
% 
%     % Create the legend for time series and mean lines
%     legend(meanHandles, meanLabels, 'Location', 'northeast', 'FontSize', 14, 'Interpreter', 'latex');
%     grid on;
%     set(gca, 'FontSize', 14); % Increase axis labels and ticks font size
% 
%     % Adjust x-axis limits to data range
%     xlim([min(cellfun(@(x) min(x), time_vector)), max(cellfun(@(x) max(x), time_vector))]);
% end