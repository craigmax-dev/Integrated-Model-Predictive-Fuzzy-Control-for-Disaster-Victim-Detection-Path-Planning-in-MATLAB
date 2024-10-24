function plotFISParallelCoordinates(fisArray)
    % Number of FIS provided
    numFIS = numel(fisArray);

    % Determine the number of outputs and membership functions (assuming all FIS have the same structure)
    numOutputs = numel(fisArray(1).Outputs);
    numMFs = numel(fisArray(1).Outputs(1).MembershipFunctions);
    
    % Create a figure
    figure;
    sgtitle('FIS Output MF Parameters', 'Interpreter', 'latex');
    colors = lines(numFIS);  % Get distinct colors for each FIS

    % Loop over each membership function index
    for mfIndex = 1:numMFs
        subplot(numMFs, 1, mfIndex);
        hold on;

        % Plot each FIS's parameters for this MF
        for f = 1:numFIS
            fis = fisArray(f);
            for i = 1:numOutputs
                mf = fis.Outputs(i).MembershipFunctions(mfIndex);
                paramsMatrix = mf.Parameters;

                % Prepare labels for parameters using the input names and LaTeX
                labels = arrayfun(@(n) strcat('$', strrep(fisArray(1).Inputs(n).Name, '_', '\_'), '$'), 1:length(mf.Parameters)-1, 'UniformOutput', false);
                labels{end+1} = '$c$';  % Label for constant term

                % Convert labels to cell array of character vectors
                labels = cellstr(labels);  % This should ensure the correct format for parallelcoords

                parallelcoords(paramsMatrix, 'Labels', labels, 'Color', colors(f,:), 'Marker', 'o', 'DisplayName', sprintf('FIS %d', f));
            end
        end

        % Customize subplot for this MF
        title(sprintf('MF "%s"', fisArray(1).Outputs(1).MembershipFunctions(mfIndex).Name), 'Interpreter', 'latex');
        xlabel('Parameter Index', 'Interpreter', 'latex');
        ylabel('Value', 'Interpreter', 'latex');
        grid on;
        legend show;

        hold off;
    end

    % Adjust layout to prevent overlapping of titles and labels
    set(gcf, 'Name', 'FIS Output MF Parameters', 'NumberTitle', 'off');
end
