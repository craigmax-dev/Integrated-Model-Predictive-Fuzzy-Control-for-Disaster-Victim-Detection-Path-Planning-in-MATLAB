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

% % Mean results from 2, 3, 4 agent cases
% fisMeanObj = [2933.5926, 2669.0626, 2797.531];
% mpcMeanObj = [3046.6, 2838.1, 2707.0149];
% mpfcMeanObj = [2723.0, 2729.6, 2394.369];
% mpcMeanTime = [111.5, 121.0467, 62.4677];
% mpfcMeanTime = [49, 73.0357, 96.4308];

% Note
% Need to rerun 4 agent MPC scenario - optimization times too tight
% Need to rerun 3 agent case - validate results!
% Need to create plot centralised vs decentralised
% Need to create plot environment size
% Can plot relative difference between MPC & MPFC

% % % Simulation names and agent counts
% simNames = {'MPC', 'MPFC'};
% agentCounts = [2, 3, 4];
% 
% % Call the function
% plotObjectiveFunction([mpcMeanObj; mpfcMeanObj], simNames, agentCounts);
% plotObjectiveFunctionNormalized([mpcMeanObj; mpfcMeanObj], fisMeanObj, simNames, agentCounts)


% numElements = numel(data.sim_mfc);  % Get the number of elements in the struct array
% half_index = 334;  % Calculate the index for half of the series, assuming 1-based indexing
% 
% for idx = 1:numElements
%     data.sim_mfc(idx).t_hist = data.sim_mfc(idx).t_hist(1:half_index);
%     data.sim_mfc(idx).s_obj_hist = data.sim_mfc(idx).s_obj_hist(1:half_index);
%     data.sim_mfc(idx).obj_hist = data.sim_mfc(idx).obj_hist(1:half_index);
% end

