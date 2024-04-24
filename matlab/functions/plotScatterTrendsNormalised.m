function plotScatterTrendsNormalised(simResults, baselineResults, simNames, agentCounts)
    % Validate inputs
    if size(simResults, 1) ~= length(simNames) || size(simResults, 2) ~= length(agentCounts) || length(baselineResults) ~= length(agentCounts)
        error('Mismatch in dimensions of simResults and simNames or agentCounts or baselineResults');
    end
    
    % Calculate the relative differences as percentages
    relDiffs = 100 * (simResults - baselineResults) ./ baselineResults;

    % Prepare the plot
    figure;
    hold on;
    grid on;
    markers = {'o', 'x', 's', 'd', '^', 'v', '<', '>'}; % Different markers for each simulation
    colors = lines(length(simNames)); % Distinct colors for each simulation
    
    % Plot each simulation result
    for i = 1:length(simNames)
        scatter(agentCounts, relDiffs(i, :), 100, markers{i}, 'DisplayName', simNames{i}, 'MarkerEdgeColor', colors(i,:));
        
        % Fit a line to the relative difference results and plot it
        coeffs = polyfit(agentCounts, relDiffs(i, :), 1); % Linear fit
        fittedY = polyval(coeffs, agentCounts);
        plot(agentCounts, fittedY, 'Color', colors(i,:), 'LineWidth', 2);
    end
    
    % Customize plot
    legend('show');
    xlabel('Number of Agents');
    ylabel('Relative Difference in Mean Objective Function Evaluation (%)');
    title('Relative Difference in Objective Function Evaluation Across Different Simulations');
    xlim([min(agentCounts)-1, max(agentCounts)+1]); % Extend x-axis a bit for clarity
    hold off;
end
