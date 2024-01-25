
% % V2.2
% function plotStats(means, ci_lower, ci_upper, time_vector, simulationSetups, titleStr, yLabel, events)
%     figure;
%     title(titleStr);
%     xlabel('Time (s)');
%     ylabel(yLabel);
%     hold on;
% 
%     % Plot the statistics
%     for simSetup = 1:size(simulationSetups, 1)
%         simulationName = simulationSetups{simSetup, 1};
%         plot(time_vector{simSetup}, means{simSetup}, 'DisplayName', strcat(simulationName, ' Mean'));
%         plot(time_vector{simSetup}, ci_lower{simSetup}, 'DisplayName', strcat(simulationName, ' Lower CI'), 'LineStyle', '--');
%         plot(time_vector{simSetup}, ci_upper{simSetup}, 'DisplayName', strcat(simulationName, ' Upper CI'), 'LineStyle', '--');
%     end
% 
%     % Plot the vertical lines for events
%     for i = 1:length(events)
%         xline(events(i).time, '-', events(i).label);
%     end
% 
%     legend('show');
%     grid on;
%     hold off;
% end

function plotStats(means, ci_lower, ci_upper, time_vector, simulationSetups, titleStr, yLabel, battery)
    figure;
    title(titleStr);
    xlabel('Time (s)');
    ylabel(yLabel);
    hold on;

    for simSetup = 1:size(simulationSetups, 1)
        simulationName = simulationSetups{simSetup, 1};
        color = rand(1,3); % Generate a random color for each simulation setup

        % Plot means, CI lower, and CI upper
        plot(time_vector{simSetup}, means{simSetup}, 'DisplayName', strcat(simulationName, ' Mean'), 'LineStyle', '--', 'Color', color);
        plot(time_vector{simSetup}, ci_lower{simSetup}, 'DisplayName', strcat(simulationName, ' Lower CI'), 'LineStyle', '--', 'Color', color);
        plot(time_vector{simSetup}, ci_upper{simSetup}, 'DisplayName', strcat(simulationName, ' Upper CI'), 'LineStyle', '--', 'Color', color);

        % Calculate and display the overall mean value
        overall_mean = mean(means{simSetup}, 'omitnan');
        yline(overall_mean, '-', strcat(simulationName, ' Overall Mean: ', num2str(overall_mean)), 'Color', color, 'LabelHorizontalAlignment', 'left', 'LineWidth', 1.5);

        % Optionally plot the means for each segment defined in the battery vector
        if nargin > 7 && ~isempty(battery)
            for i = 1:length(battery)
                % Find indices for the current segment
                segment_indices = time_vector{simSetup} <= battery(i);
                % Calculate and plot the segment mean
                segment_mean = mean(means{simSetup}(segment_indices), 'omitnan');
                yline(segment_mean, ':', strcat(simulationName, ' Segment ', num2str(i), ' Mean: ', num2str(segment_mean)), 'Color', color, 'LabelHorizontalAlignment', 'left', 'LineWidth', 1.2);
            end
        end
    end

    legend('show');
    grid on;
end


% function plotStats(means, ci_lower, ci_upper, time_vector, simulationSetups, titleStr, yLabel)
%     figure;
%     title(titleStr);
%     xlabel('Time (s)');
%     ylabel(yLabel);
%     hold on;
% 
%     for simSetup = 1:size(simulationSetups, 1)
%         simulationName = simulationSetups{simSetup, 1};
%         plot(time_vector{simSetup}, means{simSetup}, 'DisplayName', strcat(simulationName, ' Mean'));
%         plot(time_vector{simSetup}, ci_lower{simSetup}, 'DisplayName', strcat(simulationName, ' Lower CI'), 'LineStyle', '--');
%         plot(time_vector{simSetup}, ci_upper{simSetup}, 'DisplayName', strcat(simulationName, ' Upper CI'), 'LineStyle', '--');
% 
%         % Calculate and plot the overall mean of the 'means' for this simulation setup
%         overall_mean = mean(means{simSetup}, 'omitnan'); % Calculate the mean, omitting NaN values
%         yline(overall_mean, 'r-', strcat(simulationName, ' Overall Mean')); % Plot a horizontal line at the overall mean value
%     end
% 
%     legend('show');
%     grid on;
% end

