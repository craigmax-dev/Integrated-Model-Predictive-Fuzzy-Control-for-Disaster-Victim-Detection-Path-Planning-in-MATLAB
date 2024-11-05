%% V2
function plotScatterTrends(simResults, simNames, xParameter, polyOrder, confLower, confUpper, xLabel, yLabel, lineStyles)
    % Validate inputs
    if size(simResults, 1) ~= length(simNames) || size(simResults, 2) ~= length(xParameter)
        error('Mismatch in dimensions of simResults and simNames or agentCounts');
    end

    % Prepare the plot
    figure;
    hold on;
    grid on;

    % Plot each simulation result
    for i = 1:length(simNames)
        % Get the line style for the current simulation
        lineSpec = lineStyles{i};

        % Plot mean points with error bars
        errorbar(xParameter, simResults(i, :), simResults(i, :) - confLower(i, :), confUpper(i, :) - simResults(i, :), ...
            'o', 'MarkerEdgeColor', lineSpec{2}, 'MarkerFaceColor', lineSpec{2}, 'LineWidth', 1, 'Color', lineSpec{2}, 'HandleVisibility', 'off');

        % Fit a polynomial to the results
        coeffs = polyfit(xParameter, simResults(i, :), polyOrder); % Polynomial fit

        % Create a finer x grid for smooth plotting
        fineX = linspace(min(xParameter), max(xParameter), 100); % 100 points for smooth curve
        fittedY = polyval(coeffs, fineX);

        % Plot the smooth polynomial curve
        plot(fineX, fittedY, 'LineStyle', lineSpec{1}, 'Color', lineSpec{2}, 'LineWidth', 2, 'DisplayName', simNames{i});

        % Display the equation of the line of best fit
        equationText = ['y = ', sprintf('%.2f', coeffs(1))];
        for j = 2:length(coeffs)
            equationText = [equationText, sprintf(' + %.2f x^%d', coeffs(j), length(coeffs) - j)];
        end

        % Choose a central point for the equation display
        yRange = ylim; % Get the current y-axis limits
        xLoc = mean(xParameter); % Central x position
        yLoc = yRange(1) + 0.1 * (yRange(2) - yRange(1)); % Slightly above the bottom of the plot
        text(xLoc, yLoc, equationText, 'Color', lineSpec{2}, 'Interpreter', 'tex', 'HorizontalAlignment', 'center');
    end

    % Customize plot
    legend('show');
    xlabel(xLabel, 'Interpreter', 'latex');
    ylabel(yLabel, 'Interpreter', 'latex');

    % Automatically set x-axis limits with slight margins
    xMargin = 0.05 * (max(xParameter) - min(xParameter)); % Define margin as 5% of the range
    xlim([min(xParameter) - xMargin, max(xParameter) + xMargin]);

    hold off;
end

% % means_obj
% 
% % Calculate the means of the objective function values
% m_obj = cellfun(@mean, means_obj);
% m_ci_lower_obj = cellfun(@mean, ci_lower_obj);
% m_ci_upper_obj = cellfun(@mean, ci_upper_obj);
% 
% % Calculate the means of the optimization times
% m_t_opt = cellfun(@mean, means_t_opt);
% m_ci_lower_t_opt = cellfun(@mean, ci_lower_t_opt);
% m_ci_upper_t_opt = cellfun(@mean, ci_upper_t_opt);
% 
% % Display the results
% disp('Means of Objective Function Values:');
% disp(m_obj);
% 
% disp('Confidence Interval Lower Bounds for Objective Function Values:');
% disp(m_ci_lower_obj);
% 
% disp('Confidence Interval Upper Bounds for Objective Function Values:');
% disp(m_ci_upper_obj);
% 
% disp('Means of Optimization Times:');
% disp(m_t_opt);
% 
% disp('Confidence Interval Lower Bounds for Optimization Times:');
% disp(m_ci_lower_t_opt);
% 
% disp('Confidence Interval Upper Bounds for Optimization Times:');
% disp(m_ci_upper_t_opt);
% 
% % Define the threshold
% threshold = 1e3;
% 
% % Function to filter out values above the threshold
% filter_function = @(x) x(abs(x) <= threshold);