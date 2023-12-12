function [means, ci_lower, ci_upper, time_vector] = calculateStats(allResults, simulationSetups, paramName, dt_s, alpha)
    numSetups = size(simulationSetups, 1);
    means = cell(numSetups, 1);
    ci_lower = cell(numSetups, 1);
    ci_upper = cell(numSetups, 1);
    time_vector = {};

    for simSetup = 1:numSetups
        simulationName = simulationSetups{simSetup, 1};
        param_data = allResults.(simulationName).(paramName);

        mean_param = mean(param_data, 3);
        std_param = std(param_data, 0, 3);
        numIterations = size(param_data, 3);
        ci_half_width = tinv(1 - alpha/2, numIterations - 1) * std_param / sqrt(numIterations);

        means{simSetup} = mean_param;
        ci_lower{simSetup} = mean_param - ci_half_width;
        ci_upper{simSetup} = mean_param + ci_half_width;
        time_vector{simSetup} = (1:size(mean_param, 2)) * dt_s;
    end
end
