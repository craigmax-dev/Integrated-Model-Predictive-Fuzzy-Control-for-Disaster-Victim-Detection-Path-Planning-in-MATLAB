
% V2.1
function [means, ci_lower, ci_upper, time_vector] = calculateStats(allResults, simulationSetups, paramName, dt_s, alpha)
    numSetups = size(simulationSetups, 1);
    means = cell(numSetups, 1);
    ci_lower = cell(numSetups, 1);
    ci_upper = cell(numSetups, 1);
    time_vector = cell(numSetups, 1);

    for simSetup = 1:numSetups
        simulationName = simulationSetups{simSetup, 1};
        iterationsData = allResults.(simulationName);
        numIterations = numel(iterationsData);
        
        % Assuming all iterations have the same length of time series
        numTimePoints = size(iterationsData(1).(paramName), 2);
        
        aggregatedParamData = zeros(numIterations, numTimePoints);
        
        for iter = 1:numIterations
            % Collect data for this parameter across all iterations
            aggregatedParamData(iter, :) = iterationsData(iter).(paramName);
        end
        
        % Calculate the mean across iterations for each time point
        mean_param = mean(aggregatedParamData, 1);
        
        % Standard deviation across iterations for each time point
        std_param = std(aggregatedParamData, 0, 1);
        
        % Confidence interval half-width for each time point
        ci_half_width = tinv(1 - alpha/2, numIterations - 1) * std_param / sqrt(numIterations);
        
        means{simSetup} = mean_param;
        ci_lower{simSetup} = mean_param - ci_half_width;
        ci_upper{simSetup} = mean_param + ci_half_width;
        
        % Assuming t_hist is consistent across iterations or represented by a consistent time step (dt_s)
        % and starts from t=0 for each simulation setup
        time_vector{simSetup} = (0:(numTimePoints - 1)) * dt_s;
    end
end