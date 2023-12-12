function plotStats(means, ci_lower, ci_upper, time_vector, simulationSetups, titleStr, yLabel)
    figure;
    title(titleStr);
    xlabel('Time (s)');
    ylabel(yLabel);
    hold on;

    for simSetup = 1:size(simulationSetups, 1)
        simulationName = simulationSetups{simSetup, 1};
        plot(time_vector{simSetup}, means{simSetup}, 'DisplayName', strcat(simulationName, ' Mean'));
        plot(time_vector{simSetup}, ci_lower{simSetup}, 'DisplayName', strcat(simulationName, ' Lower CI'), 'LineStyle', '--');
        plot(time_vector{simSetup}, ci_upper{simSetup}, 'DisplayName', strcat(simulationName, ' Upper CI'), 'LineStyle', '--');
    end

    legend('show');
    grid on;
end
