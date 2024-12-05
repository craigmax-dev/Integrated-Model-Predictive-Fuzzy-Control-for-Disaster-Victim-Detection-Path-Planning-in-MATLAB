function plotScatterTrendsNormalised(simResults, baselineResults, simNames, xParameter, polyOrder, confLower, confUpper, xLabel, yLabel, lineStyles)
    % Validate inputs
    if size(simResults, 1) ~= length(simNames) || size(simResults, 2) ~= length(xParameter) || length(baselineResults) ~= length(xParameter)
        error('Mismatch in dimensions of simResults and simNames or xParameter or baselineResults');
    end

    % Calculate the relative differences as percentages
    relDiffs = 100 * (simResults - baselineResults) ./ baselineResults;
    confLower = 100 * (confLower - baselineResults) ./ baselineResults;
    confUpper = 100 * (confUpper - baselineResults) ./ baselineResults;

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
        errorbar(xParameter, relDiffs(i, :), relDiffs(i, :) - confLower(i, :), confUpper(i, :) - relDiffs(i, :), ...
            'LineStyle', 'none', 'Marker', marker, 'MarkerEdgeColor', color, 'MarkerFaceColor', color, ...
            'Color', color, 'LineWidth', 1, 'CapSize', capSize, 'DisplayName', simNames{i});

        % Fit a polynomial to the relative difference results
        coeffs = polyfit(xParameter, relDiffs(i, :), polyOrder);

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
    xMargin = 0.1 * (max(xParameter) - min(xParameter)); % Define margin as 10% of the range
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
