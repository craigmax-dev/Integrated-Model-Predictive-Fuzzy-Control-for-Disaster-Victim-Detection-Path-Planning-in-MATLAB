function plotScatterTrends(simResults, simNames, xParameter, polyOrder, confLower, confUpper, xLabel, yLabel, lineStyles)
    % Validate inputs
    if size(simResults, 1) ~= length(simNames) || size(simResults, 2) ~= length(xParameter)
        error('Mismatch in dimensions of simResults and simNames or xParameter');
    end

    % Prepare the plot with specified size
    figHandle = figure;
    set(figHandle, 'Units', 'centimeters', 'Position', [1, 1, 25, 20]);
    hold on;
    grid on;

    % Plot each simulation result
    for i = 1:length(simNames)
        % Get the line style for the current simulation
        lineSpec = lineStyles{i};
        lineStyle = lineSpec{1};
        colorHex = lineSpec{2};
        color = hex2rgb(colorHex);

        % Define marker and error bar styles based on line style
        switch lineStyle
            case '-'
                marker = 'o';
                capSize = 12;
            case '--'
                marker = 's';
                capSize = 6;
        end

        % Plot mean points with error bars using custom line styles
        errorbar(xParameter, simResults(i, :), simResults(i, :) - confLower(i, :), confUpper(i, :) - simResults(i, :), ...
            'LineStyle', 'none', 'Marker', marker, 'MarkerEdgeColor', color, 'MarkerFaceColor', color, ...
            'Color', color, 'LineWidth', 1, 'CapSize', capSize, 'DisplayName', simNames{i});

        % Fit a polynomial to the results
        coeffs = polyfit(xParameter, simResults(i, :), polyOrder);

        % Create a finer x grid for smooth plotting
        fineX = linspace(min(xParameter), max(xParameter), 100); % 100 points for smooth curve
        fittedY = polyval(coeffs, fineX);

        % Plot the smooth polynomial curve without adding it to the legend
        plot(fineX, fittedY, 'LineStyle', lineStyle, 'Color', color, 'LineWidth', 2, 'HandleVisibility', 'off');
    end

    % Customize plot
    legend('show', 'Location', 'best');
    xlabel(xLabel, 'Interpreter', 'latex', 'FontSize', 14);
    ylabel(yLabel, 'Interpreter', 'latex', 'FontSize', 14);
    set(gca, 'FontSize', 12);

    % Automatically set x-axis limits with slight margins
    xMargin = 0.05 * (max(xParameter) - min(xParameter)); % Define margin as 5% of the range
    xlim([min(xParameter) - xMargin, max(xParameter) + xMargin]);

    hold off;
end

% Helper function to convert hex color codes to RGB
function rgb = hex2rgb(hex)
    hex = char(hex);
    if hex(1) == '#'
        hex(1) = [];
    end
    if numel(hex) ~= 6
        error('hex color code must be 6 characters');
    end
    r = double(hex2dec(hex(1:2))) / 255;
    g = double(hex2dec(hex(3:4))) / 255;
    b = double(hex2dec(hex(5:6))) / 255;
    rgb = [r, g, b];
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