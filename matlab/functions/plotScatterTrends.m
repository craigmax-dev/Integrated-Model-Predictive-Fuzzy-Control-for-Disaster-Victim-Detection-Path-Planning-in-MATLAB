function plotScatterTrends(simResults, simNames, agentCounts)
    % Validate inputs
    if size(simResults, 1) ~= length(simNames) || size(simResults, 2) ~= length(agentCounts)
        error('Mismatch in dimensions of simResults and simNames or agentCounts');
    end
    
    % Prepare the plot
    figure;
    hold on;
    grid on;
    markers = {'o', 'x', 's', 'd', '^', 'v', '<', '>'}; % Different markers for each simulation
    colors = lines(length(simNames)); % Distinct colors for each simulation
    
    % Plot each simulation result
    for i = 1:length(simNames)
        scatter(agentCounts, simResults(i, :), 100, markers{i}, 'DisplayName', simNames{i}, 'MarkerEdgeColor', colors(i,:));
        
        % Fit a line to the results and plot it
        coeffs = polyfit(agentCounts, simResults(i, :), 1); % Linear fit
        fittedY = polyval(coeffs, agentCounts);
        plot(agentCounts, fittedY, 'Color', colors(i,:), 'LineWidth', 2);
    end
    
    % Customize plot
    legend('show');
    xlabel('Number of Agents');
    ylabel('Mean Objective Function Evaluation');
    title('Objective Function Evaluation Across Different Simulations');
    xlim([min(agentCounts)-1, max(agentCounts)+1]); % Extend x-axis a bit for clarity
    hold off;
end

% Obj V4 results
% % Mean results from 2, 3, 4 agent cases
% fisMeanObj = [2551, 2262, 2176];
% mpcMeanObj = [2619, 2312, 2044];
% mpfcCentralisedMeanObj = [2321, 2017, 1849];
% mpfcDecentralisedMeanObj = [2264, 2033, 1807];
% mpcMeanTime = [125.4, 125.6, 147.8];
% mpfcCentralisedMeanTime = [48.2, 70.7, 98.9];
% mpfcDecentralisedMeanTime = [47.1, 71.3, 93.5];

% Obj V3 results
% % Mean results from 2, 3, 4 agent cases
% fisMeanObj = [2933.5926, 2669.0626, 2797.531];
% mpcMeanObj = [3046.6, 2838.1, 2707.0149];
% mpfcMeanObj = [2723.0, 2729.6, 2394.369];
% mpcMeanTime = [111.5, 121.0467, 62.4677];
% mpfcMeanTime = [49, 73.0357, 96.4308];
% 
% % % % Simulation names and agent counts
% simNames = {'MPC', 'MPFC Centralised', 'MPFC Decentralised'};
% agentCounts = [2, 3, 4];
% 
% % Call the function
% plotScatterTrends([mpcMeanTime; mpfcCentralisedMeanTime; mpfcDecentralisedMeanTime], simNames, agentCounts);
% plotScatterTrendsNormalised([mpcMeanObj; mpfcCentralisedMeanObj; mpfcDecentralisedMeanObj], fisMeanObj, simNames, agentCounts)

