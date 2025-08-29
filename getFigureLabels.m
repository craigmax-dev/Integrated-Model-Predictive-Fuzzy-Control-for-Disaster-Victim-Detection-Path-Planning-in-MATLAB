% This script finds all open figures and prints their title, axis, and legend labels.

% Get a handle to all currently open figures.
allFigures = findall(0, 'type', 'figure');

% Check if any figures are open.
if isempty(allFigures)
    fprintf('No figures are currently open.\n');
else
    fprintf('--- Found %d open figure(s) ---\n', length(allFigures));

    % Loop through each figure to print titles and axis labels.
    for i = 1:length(allFigures)
        currentFig = allFigures(i);
        fprintf('\nProcessing Figure %d...\n', currentFig.Number);

        % Get all axes (plots and subplots) in the current figure.
        allAxes = findall(currentFig, 'type', 'axes');

        if isempty(allAxes)
            fprintf('  - This figure contains no plots.\n');
        else
            % Loop through each axis.
            for j = 1:length(allAxes)
                currentAxes = allAxes(j);
                
                % Print title and axis labels for the current plot.
                titleStr = currentAxes.Title.String;
                xLabelStr = currentAxes.XLabel.String;
                yLabelStr = currentAxes.YLabel.String;

                fprintf('  - Subplot %d:\n', j);
                fprintf('    Title: "%s"\n', titleStr);
                fprintf('    X-Axis Label: "%s"\n', xLabelStr);
                fprintf('    Y-Axis Label: "%s"\n', yLabelStr);
            end
        end

        % Now, find and print all legend labels for the current figure.
        hLegends = findobj(currentFig, 'Type', 'Legend');
        if ~isempty(hLegends)
            fprintf('\n  Legend Labels for Figure %d:\n', currentFig.Number);
            % Loop through each found legend object.
            for k = 1:length(hLegends)
                legendStrings = hLegends(k).String;
                for m = 1:length(legendStrings)
                    fprintf('      - "%s"\n', legendStrings{m});
                end
            end
        else
            fprintf('\n  Legend: (none)\n');
        end
    end
end
fprintf('\n--- Script finished. ---\n');