% V2
function plotScatterTrendsNormalised(simResults, baselineResults, simNames, xParameter, polyOrder, confLower, confUpper, xLabel, yLabel)
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
    markers = {'o', 'x', 's', 'd', '^', 'v', '<', '>'}; % Different markers for each simulation
    colors = lines(length(simNames)); % Distinct colors for each simulation

    % Plot each simulation result
    for i = 1:length(simNames)
        % Plot mean points with error bars
        errorbar(xParameter, relDiffs(i, :), relDiffs(i, :) - confLower(i, :), confUpper(i, :) - relDiffs(i, :), 'o', 'Color', colors(i,:), 'HandleVisibility', 'off');

        % Fit a polynomial to the relative difference results
        coeffs = polyfit(xParameter, relDiffs(i, :), polyOrder);
        
        % Create a finer x grid for smooth plotting
        fineX = linspace(min(xParameter), max(xParameter), 100); % 100 points for smooth curve
        fittedY = polyval(coeffs, fineX);

        % Plot the smooth polynomial curve
        plot(fineX, fittedY, 'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', simNames{i});
    end
    
    % Customize plot
    legend('show', 'Location', 'best');
    xlabel(xLabel,'Interpreter','latex');
    ylabel(yLabel,'Interpreter','latex');

    % Automatically set x-axis limits with slight margins
    xMargin = 0.1 * (max(xParameter) - min(xParameter)); % Define margin as 10% of the range
    xlim([min(xParameter) - xMargin, max(xParameter) + xMargin]);

    hold off;
end
