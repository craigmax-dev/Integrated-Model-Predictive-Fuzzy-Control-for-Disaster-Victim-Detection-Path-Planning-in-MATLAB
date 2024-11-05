function plotScatterTrendsNormalised(simResults, baselineResults, simNames, xParameter, polyOrder, confLower, confUpper, xLabel, yLabel, lineStyles)
    % Validate inputs
    if size(simResults, 1) ~= length(simNames) || size(simResults, 2) ~= length(xParameter) || length(baselineResults) ~= length(xParameter)
        error('Mismatch in dimensions of simResults and simNames or xParameter or baselineResults');
    end

    % Calculate the relative differences as percentages
    relDiffs = 100 * (simResults - baselineResults) ./ baselineResults;
    confLower = 100 * (confLower - baselineResults) ./ baselineResults;
    confUpper = 100 * (confUpper - baselineResults) ./ baselineResults;

    % Prepare the plot
    figure;
    hold on;
    grid on;

    % Helper function to convert hexadecimal to RGB
    hex2rgb = @(hex) sscanf(char(hex(2:end)), '%2x%2x%2x', [1 3]) / 255;

    % Plot each simulation result
    for i = 1:length(simNames)
        % Get the line style for the current simulation
        lineSpec = lineStyles{i};

        % Plot mean points with error bars
        errorbar(xParameter, relDiffs(i, :), relDiffs(i, :) - confLower(i, :), confUpper(i, :) - relDiffs(i, :), ...
            'o', 'MarkerEdgeColor', lineSpec{2}, 'MarkerFaceColor', lineSpec{2}, 'LineWidth', 1, 'Color', lineSpec{2}, 'HandleVisibility', 'off');

        % Fit a polynomial to the relative difference results
        coeffs = polyfit(xParameter, relDiffs(i, :), polyOrder);

        % Create a finer x grid for smooth plotting
        fineX = linspace(min(xParameter), max(xParameter), 100); % 100 points for smooth curve
        fittedY = polyval(coeffs, fineX);

        % Plot the smooth polynomial curve
        plot(fineX, fittedY, 'LineStyle', lineSpec{1}, 'Color', lineSpec{2}, 'LineWidth', 2, 'DisplayName', simNames{i});
    end
    
    % Customize plot
    legend('show', 'Location', 'best');
    xlabel(xLabel, 'Interpreter', 'latex');
    ylabel(yLabel, 'Interpreter', 'latex');

    % Automatically set x-axis limits with slight margins
    xMargin = 0.1 * (max(xParameter) - min(xParameter)); % Define margin as 10% of the range
    xlim([min(xParameter) - xMargin, max(xParameter) + xMargin]);

    hold off;
end
