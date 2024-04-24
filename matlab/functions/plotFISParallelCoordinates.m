function plotFISParallelCoordinates(fis)
    % Total number of membership functions across all outputs
    totalMFs = sum(arrayfun(@(out) numel(out.MembershipFunctions), fis.Outputs));

    % Create a figure
    figure;
    sgtitle('Parallel Coordinates Plot for Each Output Membership Function');

    % Counter for subplots
    mfCounter = 1;

    % Loop over each output
    for i = 1:numel(fis.Outputs)
        % Loop over each membership function in the current output
        for j = 1:numel(fis.Outputs(i).MembershipFunctions)
            subplot(totalMFs, 1, mfCounter);
            hold on;

            % Get the membership function
            mf = fis.Outputs(i).MembershipFunctions(j);
            paramsMatrix = mf.Parameters;

            % Prepare labels for parameters
            paramCount = numel(mf.Parameters);
            labels = arrayfun(@(n) sprintf('Param %d', n), 1:paramCount, 'UniformOutput', false);

            % Plot the parameters of the membership function
            parallelcoords(paramsMatrix, 'Labels', labels, 'Marker', 'o');

            % Customize subplot
            title(sprintf('Output %s - MF "%s"', fis.Outputs(i).Name, mf.Name));
            xlabel('Parameters');
            ylabel('Parameter Value');
            grid on;

            hold off;

            % Increment the membership function counter
            mfCounter = mfCounter + 1;
        end
    end

    % Adjust layout to prevent overlapping of titles and labels
    set(gcf, 'Name', 'FIS Output MF Parameters', 'NumberTitle', 'off');
end
