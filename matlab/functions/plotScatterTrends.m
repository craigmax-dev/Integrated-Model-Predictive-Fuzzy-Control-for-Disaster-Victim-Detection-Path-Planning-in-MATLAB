%% V2
function plotScatterTrends(simResults, simNames, xParameter, polyOrder, confLower, confUpper, xLabel, yLabel, lineStyles)
    % Validate inputs
    if size(simResults, 1) ~= length(simNames) || size(simResults, 2) ~= length(xParameter) || length(lineStyles) ~= length(simNames)
        error('Mismatch in the dimensions of inputs.');
    end

    % Prepare the plot
    figure;
    hold on;
    grid on;

    % Plot each simulation result
    for i = 1:length(simNames)
        % Error bars setup
        errorbar(xParameter, simResults(i, :), confLower(i, :), confUpper(i, :), ...
            'o', 'MarkerEdgeColor', lineStyles(i).color, 'MarkerFaceColor', lineStyles(i).color, ...
            'Color', lineStyles(i).color, 'LineWidth', 1, 'HandleVisibility', 'off');

        % Polynomial fit and plotting
        coeffs = polyfit(xParameter, simResults(i, :), polyOrder);
        fineX = linspace(min(xParameter), max(xParameter), 100);
        fittedY = polyval(coeffs, fineX);
        plot(fineX, fittedY, 'LineStyle', lineStyles(i).style, 'Color', lineStyles(i).color, 'LineWidth', 2, 'DisplayName', simNames{i});

        % Display the equation of the line of best fit
        equationStr = sprintf('y = %.2f', coeffs(1));
        for j = 2:length(coeffs)
            equationStr = [equationStr, sprintf(' + %.2f x^{%d}', coeffs(j), length(coeffs) - j)];
        end
        % Positioning the equation text
        yRange = ylim;
        xLoc = mean(xParameter); % Central x position
        yLoc = yRange(1) + 0.1 * (yRange(2) - yRange(1)); % 10% above the bottom
        text(xLoc, yLoc, equationStr, 'Color', lineStyles(i).color, 'Interpreter', 'latex', 'HorizontalAlignment', 'center');
    end

    % Customize plot
    legend('show', 'Location', 'best');
    xlabel(xLabel, 'Interpreter', 'latex');
    ylabel(yLabel, 'Interpreter', 'latex');
    xMargin = 0.05 * (max(xParameter) - min(xParameter));
    xlim([min(xParameter) - xMargin, max(xParameter) + xMargin]);
    hold off;
end


%% V1
% function plotScatterTrends(simResults, simNames, xParameter, polyOrder, confLower, confUpper, xLabel, yLabel)
%     % Validate inputs
%     if size(simResults, 1) ~= length(simNames) || size(simResults, 2) ~= length(xParameter)
%         error('Mismatch in dimensions of simResults and simNames or agentCounts');
%     end
% 
%     % Prepare the plot
%     figure;
%     hold on;
%     grid on;
%     markers = {'o', 'x', 's', 'd', '^', 'v', '<', '>'}; % Different markers for each simulation
%     colors = lines(length(simNames)); % Distinct colors for each simulation
% 
%     % Plot each simulation result
%     for i = 1:length(simNames)
%         % Plot mean points with error bars
%         errorbar(xParameter, simResults(i, :), simResults(i, :) - confLower(i, :), confUpper(i, :) - simResults(i, :), ...
%             'o', 'MarkerEdgeColor', colors(i,:), 'MarkerFaceColor', colors(i,:), 'LineWidth', 1, 'Color', colors(i,:), 'HandleVisibility', 'off');
% 
%         % Fit a polynomial to the results
%         coeffs = polyfit(xParameter, simResults(i, :), polyOrder); % Polynomial fit
% 
%         % Create a finer x grid for smooth plotting
%         fineX = linspace(min(xParameter), max(xParameter), 100); % 100 points for smooth curve
%         fittedY = polyval(coeffs, fineX);
% 
%         % Plot the smooth polynomial curve
%         plot(fineX, fittedY, 'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', simNames{i});
% 
%         % Display the equation of the line of best fit
%         equationText = ['y = ', sprintf('%.2f', coeffs(1))];
%         for j = 2:length(coeffs)
%             equationText = [equationText, sprintf(' + %.2f x^%d', coeffs(j), length(coeffs) - j)];
%         end
% 
%         % Choose a central point for the equation display
%         yRange = ylim; % Get the current y-axis limits
%         xLoc = mean(xParameter); % Central x position
%         yLoc = yRange(1) + 0.1 * (yRange(2) - yRange(1)); % Slightly above the bottom of the plot
%         text(xLoc, yLoc, equationText, 'Color', colors(i,:), 'Interpreter', 'tex', 'HorizontalAlignment', 'center');
%     end
% 
%     % Customize plot
%     legend('show');
%     xlabel(xLabel,'Interpreter','latex');
%     ylabel(yLabel,'Interpreter','latex');
% 
%     % Automatically set x-axis limits with slight margins
%     xMargin = 0.05 * (max(xParameter) - min(xParameter)); % Define margin as 5% of the range
%     xlim([min(xParameter) - xMargin, max(xParameter) + xMargin]);
% 
%     hold off;
% end
% 
% % means_obj
% 
% % % Calculate the means of the objective function values
% % m_obj = cellfun(@mean, means_obj);
% % m_ci_lower_obj = cellfun(@mean, ci_lower_obj);
% % m_ci_upper_obj = cellfun(@mean, ci_upper_obj);
% % 
% % % Calculate the means of the optimization times
% % m_t_opt = cellfun(@mean, means_t_opt);
% % m_ci_lower_t_opt = cellfun(@mean, ci_lower_t_opt);
% % m_ci_upper_t_opt = cellfun(@mean, ci_upper_t_opt);
% % 
% % % Display the results
% % disp('Means of Objective Function Values:');
% % disp(m_obj);
% % 
% % disp('Confidence Interval Lower Bounds for Objective Function Values:');
% % disp(m_ci_lower_obj);
% % 
% % disp('Confidence Interval Upper Bounds for Objective Function Values:');
% % disp(m_ci_upper_obj);
% % 
% % disp('Means of Optimization Times:');
% % disp(m_t_opt);
% % 
% % disp('Confidence Interval Lower Bounds for Optimization Times:');
% % disp(m_ci_lower_t_opt);
% % 
% % disp('Confidence Interval Upper Bounds for Optimization Times:');
% % disp(m_ci_upper_t_opt);
% % 
% % % Define the threshold
% % threshold = 1e3;
% % 
% % % Function to filter out values above the threshold
% % filter_function = @(x) x(abs(x) <= threshold);
% 
% %% structures - sensitivity analysis for the results using structures (note no dynamics)
% 
% %% prediction mode
% 
% % probThresh_MeanObj = 1.0e+03*[2.1525, 2.2396];
% % probThresh_MeanObj_confLower = 1.0e+03 *[1.9411, 1.9858];
% % probThresh_MeanObj_confUpper = 1.0e+03 *[2.3640, 2.4935];
% % probThresh_MeanTime = [38.9131, 105.2248];
% % probThresh_MeanTime_confLower = [31.1143, 86.2618];
% % probThresh_MeanTime_confUpper = [46.7118, 124.1878];
% % 
% % exact_MeanObj = 1.0e+03*[2.0296, 2.0608];
% % exact_MeanObj_confLower = 1.0e+03 *[1.8295, 1.9253];
% % exact_MeanObj_confUpper = 1.0e+03 *[2.2296, 2.1963];
% % exact_MeanTime = [42.8821, 100.1935];
% % exact_MeanTime_confLower = [29.2213, 83.0120];
% % exact_MeanTime_confUpper = [56.5428, 117.3750];
% % 
% % predMode_MeanObj            = [probThresh_MeanObj           ; exact_MeanObj           ];
% % predMode_MeanObj_confLower  = [probThresh_MeanObj_confLower ; exact_MeanObj_confLower ];
% % predMode_MeanObj_confUpper  = [probThresh_MeanObj_confUpper ; exact_MeanObj_confUpper ];
% % predMode_MeanTime           = [probThresh_MeanTime          ; exact_MeanTime          ];
% % predMode_MeanTime_confLower = [probThresh_MeanTime_confLower; exact_MeanTime_confLower];
% % predMode_MeanTime_confUpper = [probThresh_MeanTime_confUpper; exact_MeanTime_confUpper];
% % 
% % 
% % simNames = {'Probability Threshold', 'Exact'};
% % t_mpc = 15*[30, 60];
% % 
% % % Call the functions
% 
% % plotScatterTrends(predMode_MeanObj, simNames, t_mpc, 1, predMode_MeanObj_confLower, predMode_MeanObj_confUpper, "MPC Timestep, $\Delta t^{\mathrm{MPC}}$", "Objective Function, $\overline{J}$");
% % plotScatterTrends(predMode_MeanTime, simNames, t_mpc, 1, predMode_MeanTime_confLower, predMode_MeanTime_confUpper, "MPC Timestep, $\Delta t^{\mathrm{MPC}}$", "Optimisation Time, $\overline{t}^{\mathrm{opt}}$");
% 
% % simNames = {'Probability Threshold'};
% % plotScatterTrendsNormalised(probThresh_MeanObj, exact_MeanObj, simNames, t_mpc, 1, probThresh_MeanObj_confLower, probThresh_MeanObj_confUpper, "MPC Timestep, $\Delta t^{\mathrm{MPC}}$", "Normalised Objective Function, $\overline{J}$");
% 
% %% n_a
% 
% % % % Data
% % fisMeanObj = 1.0e+03 *[2.5506, 2.2616, 2.1763];
% % 
% % mpfcCentralisedMeanTime = [48.2061, 70.7112, 96.8939];
% % mpfcCentralisedMeanTime_confLower = [47.4827, 68.4695, 94.7810];
% % mpfcCentralisedMeanTime_confUpper = [48.9295, 72.9529, 99.0068];
% % mpfcCentralisedMeanObj = 1.0e+03 *[2.3214, 2.0171, 1.8487];
% % mpfcCentralisedMeanObj_confLower = 1.0e+03 *[2.0998, 1.7781, 1.5523];
% % mpfcCentralisedMeanObj_confUpper = 1.0e+03 *[2.5430, 2.2561, 2.1451];
% % 
% % mpfcDecentralisedMeanTime = [23.5540, 23.7515, 23.3633];
% % mpfcDecentralisedMeanTime_confLower = [23.3618, 20.6987, 23.1581];
% % mpfcDecentralisedMeanTime_confUpper = [23.7462, 26.8044, 23.5684];
% % mpfcDecentralisedMeanObj = 1.0e+03 *[2.2642, 2.0330, 1.8070];
% % mpfcDecentralisedMeanObj_confLower = 1.0e+03 *[1.9367, 1.8009, 1.5147];
% % mpfcDecentralisedMeanObj_confUpper = 1.0e+03 *[2.5918, 2.2651, 2.0993];
% % 
% % mpcCentralisedMeanTime = [125.4170, 125.6490,  147.7781];
% % mpcCentralisedMeanTime_confLower = [94.6109, 96.1807, 106.4945];
% % mpcCentralisedMeanTime_confUpper = [156.2232, 155.1172, 189.0617];
% % mpcCentralisedMeanObj = 1.0e+03 *[2.6187, 2.3124, 2.0442];
% % mpcCentralisedMeanObj_confLower = 1.0e+03 *[2.3745, 2.1004, 1.8818];
% % mpcCentralisedMeanObj_confUpper = 1.0e+03 *[2.8629, 2.5244, 2.2067];
% % 
% % mpcDecentralisedMeanTime = [124.2137, 122.6140, 129.4289];
% % mpcDecentralisedMeanTime_confLower = [91.9590, 90.1793, 90.4822];
% % mpcDecentralisedMeanTime_confUpper = [156.4684, 155.0486, 168.3756];
% % mpcDecentralisedMeanObj = 1.0e+03 *[2.3870, 2.1690, 1.9107];
% % mpcDecentralisedMeanObj_confLower = 1.0e+03 *[2.1491, 2.0529, 1.6598];
% % mpcDecentralisedMeanObj_confUpper = 1.0e+03 *[2.6250, 2.2850, 2.1617];
% % 
% % simNames = {'Centralised MPC', 'Decentralised MPC', 'Centralised MPFC', 'Decentralised MPFC'};
% % agentCounts = [2, 3, 4];
% % 
% % 
% % % Call the functions
% % plotScatterTrends([mpcCentralisedMeanTime; mpcDecentralisedMeanTime; mpfcCentralisedMeanTime; mpfcDecentralisedMeanTime], simNames, agentCounts, 1, [mpcCentralisedMeanTime_confLower; mpcDecentralisedMeanTime_confLower; mpfcCentralisedMeanTime_confLower; mpfcDecentralisedMeanTime_confLower], [mpcCentralisedMeanTime_confUpper; mpcDecentralisedMeanTime_confUpper; mpfcCentralisedMeanTime_confUpper; mpfcDecentralisedMeanTime_confUpper], "Number of Agents, $n^{a}$", "Optimisation time, $\overline{t}^{\mathrm{opt}}$");
% % plotScatterTrendsNormalised([mpcCentralisedMeanObj; mpcDecentralisedMeanObj; mpfcCentralisedMeanObj; mpfcDecentralisedMeanObj], fisMeanObj, simNames, agentCounts, 1, [mpcCentralisedMeanObj_confLower; mpcDecentralisedMeanObj_confLower; mpfcCentralisedMeanObj_confLower; mpfcDecentralisedMeanObj_confLower], [mpcCentralisedMeanObj_confUpper; mpcDecentralisedMeanObj_confUpper; mpfcCentralisedMeanObj_confUpper; mpfcDecentralisedMeanObj_confUpper], "Number of Agents, $n^{a}$", "Normalised Objective Function, $\overline{J}$");
% 
% %% t_MPC
% % t_MPC_MeanObj = 1.0e+03*[7.1370, 7.0530, 7.2312, 7.0589, 7.7787, 8.0764];
% % t_MPC_MeanObj_confLower = 1.0e+03 *[6.6995, 6.6821, 6.7562, 6.5413, 7.2780, 7.3175];
% % t_MPC_MeanObj_confUpper = 1.0e+03 *[7.5746, 7.4238, 7.7063, 7.5765, 8.2794, 8.8353];
% % t_MPC_MeanTime = [46.0835, 54.1516, 74.7733, 121.3524, 161.9631, 202.1015];
% % t_MPC_MeanTime_confLower = [44.6674, 52.4588, 69.8264, 114.3734, 160.5813, 200.6613];
% % t_MPC_MeanTime_confUpper = [47.4997, 55.8444, 79.7201, 128.3313, 163.3449, 203.5417];
% % 
% % simNames = {'Centralised MPFC'};
% % t_mpc = 15*[2, 5, 15, 30, 45, 60];
% % 
% % % Call the functions
% % plotScatterTrends(t_MPC_MeanObj, simNames, t_mpc, 2, t_MPC_MeanObj_confLower, t_MPC_MeanObj_confUpper, "MPC Timestep, $\Delta t^{\mathrm{MPC}}$", "Objective Function, $\overline{J}$");
% % plotScatterTrends(t_MPC_MeanTime, simNames, t_mpc, 1, t_MPC_MeanTime_confLower, t_MPC_MeanTime_confUpper, "MPC Timestep, $\Delta t^{\mathrm{MPC}}$", "Optimisation time, $\overline{t}^{\mathrm{opt}}$");
% 
% %% t_pred
% % t_pred_MeanObj = 1.0e+03*[7.77, 7.7787, 7.805, 7.684];
% % t_pred_MeanObj_confLower = 1.0e+03 *[7.3310, 7.2780, 7.3690, 7.3288];
% % t_pred_MeanObj_confUpper = 1.0e+03 *[8.2091, 8.2794, 8.2409, 8.0391];
% % t_pred_MeanTime = [78.2812, 161.9631, 159.9497, 202.9531];
% % t_pred_MeanTime_confLower = [68.6694, 160.5813, 133.5439, 172.0526];
% % t_pred_MeanTime_confUpper = [87.8929, 163.3449, 186.3556, 233.8537];
% % 
% % simNames = {'Centralised MPFC'};
% % t_pred = 15*[30, 45, 60, 75];
% % 
% % % Call the functions
% % plotScatterTrends(t_pred_MeanObj, simNames, t_pred, 1, t_pred_MeanObj_confLower, t_pred_MeanObj_confUpper, "Prediction Timestep, $\Delta t^{\mathrm{pred}}$", "Objective Function, $\overline{J}$");
% % plotScatterTrends(t_pred_MeanTime, simNames, t_pred, 1, t_pred_MeanTime_confLower, t_pred_MeanTime_confUpper, "Prediction Timestep, $\Delta t^{\mathrm{pred}}$", "Optimisation time, $\overline{t}^{\mathrm{opt}}$");
% 
% %% Disaster environment size
% 
% % Data
% % fisMeanObj = 1.0e+03*[0.207, 2.1763, 7.6638];
% % 
% % mpfcCentralisedMeanTime = [21.5653, 96.8939, 168.0375];
% % mpfcCentralisedMeanTime_confLower = [20.8904, 94.7810, 165.8786];
% % mpfcCentralisedMeanTime_confUpper = [22.2401, 99.0068, 170.1964];
% % mpfcCentralisedMeanObj = 1.0e+03 *[0.1908, 1.8487, 6.9163];
% % mpfcCentralisedMeanObj_confLower = 1.0e+03 *[0.156, 1.5523, 6.4595];
% % mpfcCentralisedMeanObj_confUpper = 1.0e+03 *[0.225, 2.1451, 7.3732];
% % 
% % % mpfcDecentralisedMeanTime = [23.3633];
% % % mpfcDecentralisedMeanTime_confLower = [23.1581];
% % % mpfcDecentralisedMeanTime_confUpper = [23.5684];
% % % mpfcDecentralisedMeanObj = 1.0e+03 *[1.8070];
% % % mpfcDecentralisedMeanObj_confLower = 1.0e+03 *[1.5147];
% % % mpfcDecentralisedMeanObj_confUpper = 1.0e+03 *[2.0993];
% % 
% % mpcMeanTime = [42.9858, 147.7781, 246.8798];
% % mpcMeanTime_confLower = [31.6914, 106.4945, 189.8676];
% % mpcMeanTime_confUpper = [54.2802, 189.0617, 303.8921];
% % mpcMeanObj = 1.0e+03 *[0.2697, 2.0442, 7.7265];
% % mpcMeanObj_confLower = 1.0e+03 *[0.218, 1.8818, 7.4081];
% % mpcMeanObj_confUpper = 1.0e+03 *[0.320, 2.2067, 8.0449];
% % 
% % simNames = {'Centralised MPC', 'Centralised MPFC'};
% % % envSize = [40, 60];
% % envSize = [400, 1600, 3600];
% % 
% % % Call the functions
% % plotScatterTrends([mpcMeanTime; mpfcCentralisedMeanTime], simNames, envSize, 1, [mpcMeanTime_confLower; mpfcCentralisedMeanTime_confLower], [mpcMeanTime_confUpper; mpfcCentralisedMeanTime_confUpper], "Number of Environment Cells, $n^{env^{x}} \cdot n^{env^{y}}$", "Optimisation time, $\overline{t}^{\mathrm{opt}}$");
% % % plotScatterTrends([mpcMeanObj; mpfcCentralisedMeanObj], simNames, envSize, 1, [mpcMeanObj_confLower; mpfcCentralisedMeanObj_confLower], [mpcMeanObj_confUpper; mpfcCentralisedMeanObj_confUpper], "Number of Environment Cells, $n^{env^{x}} \cdot n^{env^{y}}$", "Normalised Objective Function, $\overline{J} (\%)$");
% % plotScatterTrendsNormalised([mpcMeanObj; mpfcCentralisedMeanObj], fisMeanObj, simNames, envSize, 1, [mpcMeanObj_confLower; mpfcCentralisedMeanObj_confLower], [mpcMeanObj_confUpper; mpfcCentralisedMeanObj_confUpper], "Number of Environment Cells, $n^{env^{x}} \cdot n^{env^{y}}$", "Normalised Objective Function, $\overline{J} (\%)$");
% % 
% % % Make the y-axis logarithmic
% % % set(gca, 'YScale', 'log');
% % 
% % % Filter the legend to keep only the rows numbered 1, 3, 5, 7, 9
% % legend_entries = legend;
% % legend_entries.String = legend_entries.String(1:2:5); % Keep only rows 1, 3, 5, 7, 9
% % legend(legend_entries.String);
% % 
% % % Remove the title
% % title('');
% % 
% % % Increase the thickness of all lines to 3
% % lines = findall(gca, 'Type', 'Line');
% % set(lines, 'LineWidth', 3);