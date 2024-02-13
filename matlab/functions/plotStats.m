% Plot timeseries parameter with stats

% TO DO
% Remove CI lines from legend when not included in plot

% V2.1
function plotStats(means, ci_lower, ci_upper, time_vector, simulationSetups, titleStr, yLabel, battery)
    figure;
    set(gcf, 'Position', [100, 100, 1049, 895]); % Optional: Set figure size
    title(titleStr, 'FontSize', 14);
    xlabel('Time (s)', 'FontSize', 12);
    ylabel(yLabel, 'FontSize', 12);
    hold on;

    % Predefined colors for consistency and clarity
    colors = [0, 0.4470, 0.7410; 0.8500, 0.3250, 0.0980; 0.9290, 0.6940, 0.1250; 0.4940, 0.1840, 0.5560; 0.4660, 0.6740, 0.1880; 0.3010, 0.7450, 0.9330; 0.6350, 0.0780, 0.1840];
    colorIndex = 1;

    for simSetup = 1:size(simulationSetups, 1)
        simulationName = simulationSetups{simSetup, 1};
        color = colors(mod(colorIndex - 1, size(colors, 1)) + 1, :); % Cycle through predefined colors
        colorIndex = colorIndex + 1;

        % Plot means
        plot(time_vector{simSetup}, means{simSetup}, 'DisplayName', strcat(simulationName, ' Mean'), 'LineStyle', '-', 'Color', color, 'LineWidth', 2);

        % Conditionally plot CI if different from mean
        if any(ci_lower{simSetup} ~= means{simSetup}) || any(ci_upper{simSetup} ~= means{simSetup})
            plot(time_vector{simSetup}, ci_lower{simSetup}, 'DisplayName', strcat(simulationName, ' Lower CI'), 'LineStyle', ':', 'Color', color, 'LineWidth', 1.5);
            plot(time_vector{simSetup}, ci_upper{simSetup}, 'DisplayName', strcat(simulationName, ' Upper CI'), 'LineStyle', ':', 'Color', color, 'LineWidth', 1.5);
        end

        % Display the overall mean value
        overall_mean = mean(means{simSetup}, 'omitnan');
        yline(overall_mean, '-', strcat(simulationName, ' Overall Mean: ', num2str(overall_mean)), 'Color', color, 'LabelHorizontalAlignment', 'left', 'LineWidth', 1.5, 'FontSize', 10);

        % Optionally plot the means for each segment defined in the battery vector
        if nargin > 7 && ~isempty(battery)
            for i = 1:length(battery)
                segment_indices = time_vector{simSetup} <= battery(i);
                segment_mean = mean(means{simSetup}(segment_indices), 'omitnan');
                yline(segment_mean, ':', strcat(simulationName, ' Segment ', num2str(i), ' Mean: ', num2str(segment_mean)), 'Color', color, 'LabelHorizontalAlignment', 'left', 'LineWidth', 1.2, 'FontSize', 10);
            end
        end
    end

    legend('show', 'Location', 'bestoutside');
    grid on;
    set(gca, 'FontSize', 10); % Make axis labels and ticks larger
end


% % V2
% function plotStats(means, ci_lower, ci_upper, time_vector, simulationSetups, titleStr, yLabel, battery)
%     figure;
%     title(titleStr);
%     xlabel('Time (s)');
%     ylabel(yLabel);
%     hold on;
% 
%     for simSetup = 1:size(simulationSetups, 1)
%         simulationName = simulationSetups{simSetup, 1};
%         color = rand(1,3); % Generate a random color for each simulation setup
% 
%         % Plot means, CI lower, and CI upper
%         plot(time_vector{simSetup}, means{simSetup}, 'DisplayName', strcat(simulationName, ' Mean'), 'LineStyle', '--', 'Color', color);
%         plot(time_vector{simSetup}, ci_lower{simSetup}, 'DisplayName', strcat(simulationName, ' Lower CI'), 'LineStyle', '--', 'Color', color);
%         plot(time_vector{simSetup}, ci_upper{simSetup}, 'DisplayName', strcat(simulationName, ' Upper CI'), 'LineStyle', '--', 'Color', color);
% 
%         % Calculate and display the overall mean value
%         overall_mean = mean(means{simSetup}, 'omitnan');
%         yline(overall_mean, '-', strcat(simulationName, ' Overall Mean: ', num2str(overall_mean)), 'Color', color, 'LabelHorizontalAlignment', 'left', 'LineWidth', 1.5);
% 
%         % Optionally plot the means for each segment defined in the battery vector
%         if nargin > 7 && ~isempty(battery)
%             for i = 1:length(battery)
%                 % Find indices for the current segment
%                 segment_indices = time_vector{simSetup} <= battery(i);
%                 % Calculate and plot the segment mean
%                 segment_mean = mean(means{simSetup}(segment_indices), 'omitnan');
%                 yline(segment_mean, ':', strcat(simulationName, ' Segment ', num2str(i), ' Mean: ', num2str(segment_mean)), 'Color', color, 'LabelHorizontalAlignment', 'left', 'LineWidth', 1.2);
%             end
%         end
%     end
% 
%     legend('show');
%     grid on;
% end