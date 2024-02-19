% Plot timeseries parameter with stats

% CHANGELOG
% Added shaded CI

% TODO
% Option to plot individual results lines

% V2.3
function plotStats(means, ci_lower, ci_upper, time_vector, simulationSetups, titleStr, yLabel, battery)
    % Make the plot fullscreen
    figure;
    set(gcf, 'Position', get(0, 'Screensize')); % Set figure to fullscreen
    
    % Increase text sizes for title, xlabel, and ylabel
    title(titleStr, 'FontSize', 18);
    xlabel('Time (s)', 'FontSize', 16);
    ylabel(yLabel, 'FontSize', 16);
    hold on;
    
    % Predefined colors for consistency and clarity
    colors = [0, 0.4470, 0.7410; 0.8500, 0.3250, 0.0980; 0.9290, 0.6940, 0.1250; 0.4940, 0.1840, 0.5560; 0.4660, 0.6740, 0.1880; 0.3010, 0.7450, 0.9330; 0.6350, 0.0780, 0.1840];
    colorIndex = 1;

    for simSetup = 1:size(simulationSetups, 1)
        simulationName = simulationSetups{simSetup, 7};
        color = colors(mod(colorIndex - 1, size(colors, 1)) + 1, :); % Cycle through predefined colors
        colorIndex = colorIndex + 1;
        
        % Plot means
        plot(time_vector{simSetup}, means{simSetup}, 'DisplayName', strcat(simulationName, ' Mean'), 'LineStyle', '-', 'Color', color, 'LineWidth', 2);
        
        % Shaded CI area
        X = [time_vector{simSetup}, fliplr(time_vector{simSetup})];
        Y = [ci_lower{simSetup}, fliplr(ci_upper{simSetup})];
        fill(X, Y, color, 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', strcat(simulationName, ' CI'));
        
        % Adjusting yline for overall mean with increased 'FontSize'
        overall_mean = mean(means{simSetup}, 'omitnan');
        yline(overall_mean, '-', strcat(simulationName, ' Overall Mean: ', num2str(overall_mean)), 'Color', color, 'LabelHorizontalAlignment', 'left', 'LineWidth', 2, 'FontSize', 14);
        
        % Optionally plot the means for each segment defined in the battery vector
        if nargin > 7 && ~isempty(battery)
            for i = 1:length(battery)
                segment_indices = time_vector{simSetup} <= battery(i);
                segment_mean = mean(means{simSetup}(segment_indices), 'omitnan');
                yline(segment_mean, ':', strcat(simulationName, ' Segment ', num2str(i), ' Mean: ', num2str(segment_mean)), 'Color', color, 'LabelHorizontalAlignment', 'left', 'LineWidth', 1.2, 'FontSize', 14);
            end
        end
    end
    
    % Position legend in the top left corner and increase font size
    legend('Location', 'northwest', 'FontSize', 14);
    grid on;
    set(gca, 'FontSize', 14); % Increase axis labels and ticks font size
end

% % V2.2
% function plotStats(means, ci_lower, ci_upper, time_vector, simulationSetups, titleStr, yLabel, battery)
%     % Make the plot fullscreen
%     figure;
%     set(gcf, 'Position', get(0, 'Screensize')); % Set figure to fullscreen
% 
%     % Increase text sizes for title, xlabel, and ylabel
%     title(titleStr, 'FontSize', 18);
%     xlabel('Time (s)', 'FontSize', 16);
%     ylabel(yLabel, 'FontSize', 16);
%     hold on;
% 
%     % Predefined colors for consistency and clarity
%     colors = [0, 0.4470, 0.7410; 0.8500, 0.3250, 0.0980; 0.9290, 0.6940, 0.1250; 0.4940, 0.1840, 0.5560; 0.4660, 0.6740, 0.1880; 0.3010, 0.7450, 0.9330; 0.6350, 0.0780, 0.1840];
%     colorIndex = 1;
% 
%     for simSetup = 1:size(simulationSetups, 1)
%         simulationName = simulationSetups{simSetup, 7};
%         color = colors(mod(colorIndex - 1, size(colors, 1)) + 1, :); % Cycle through predefined colors
%         colorIndex = colorIndex + 1;
% 
%         % Plot means and confidence intervals (CI)
%         % Similar plotting commands as before, but adjusting 'LineWidth' and 'FontSize' for visibility
%         plot(time_vector{simSetup}, means{simSetup}, 'DisplayName', strcat(simulationName, ' Mean'), 'LineStyle', '-', 'Color', color, 'LineWidth', 2);
% 
%         if any(ci_lower{simSetup} ~= means{simSetup}) || any(ci_upper{simSetup} ~= means{simSetup})
%             plot(time_vector{simSetup}, ci_lower{simSetup}, 'DisplayName', strcat(simulationName, ' Lower CI'), 'LineStyle', ':', 'Color', color, 'LineWidth', 2);
%             plot(time_vector{simSetup}, ci_upper{simSetup}, 'DisplayName', strcat(simulationName, ' Upper CI'), 'LineStyle', ':', 'Color', color, 'LineWidth', 2);
%         end
% 
%         % Adjusting yline for overall mean with increased 'FontSize'
%         overall_mean = mean(means{simSetup}, 'omitnan');
%         yline(overall_mean, '-', strcat(simulationName, ' Overall Mean: ', num2str(overall_mean)), 'Color', color, 'LabelHorizontalAlignment', 'left', 'LineWidth', 2, 'FontSize', 14);
% 
%         % Plotting battery segment means, if applicable
%         if nargin > 7 && ~isempty(battery)
%             % Similar adjustments for segment mean lines
%         end
%     end
% 
%     % Position legend in the top left corner and increase font size
%     legend('Location', 'northwest', 'FontSize', 14);
%     grid on;
%     set(gca, 'FontSize', 14); % Increase axis labels and ticks font size
% end
