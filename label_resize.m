% resize_figure_fonts.m
%
% This script opens every .fig file within a specified folder and its 
% subfolders, resizes all text elements (titles, labels, legends, etc.)
% to predefined sizes.
% MODIFIED: Figures are left open for manual review and are not saved.

% ----- USER INPUTS -----
% Specify the folder containing the images.
% Use a full path, e.g., 'C:\Users\YourName\Documents\MyImages'
folderPath = 'C:\Users\CMAX2647\Documents\GitHub\Integrated-Model-Predictive-Fuzzy-Control-for-Disaster-Victim-Detection-Path-Planning-in-MATLAB\simulations\4_2_4_centralised_vs_decentralised\2a';

% Define the font sizes for different text elements.
fontSize.axisLabels = 20;
fontSize.tickLabels = 16;
fontSize.title = 20;
fontSize.legend = 20;

% ----- SCRIPT LOGIC -----

% Check if the folder exists.
if ~isfolder(folderPath)
    error('Error: The specified folder "%s" does not exist.', folderPath);
end

% Get a list of all .fig files in the folder and its subfolders.
imageFiles = dir(fullfile(folderPath, '**', '*.fig')); 

% Check if any image files were found.
if isempty(imageFiles)
    warning('No .fig files found in the specified folder "%s".', folderPath);
    return;
end

% Loop through each found image file.
fprintf('Processing font sizes for %d image files...\n', length(imageFiles));

for i = 1:length(imageFiles)
    fileName = imageFiles(i).name;
    filePath = fullfile(imageFiles(i).folder, fileName);

    fprintf('  -> Processing: %s\n', filePath);

    % Suppress the harmless 'classNotFound' warning that can occur
    % when opening figures saved in different MATLAB versions.
    warnState = warning('off', 'MATLAB:load:classNotFound');
    try
        % MODIFICATION: Open the figure and make sure it's visible.
        hFig = openfig(filePath, 'reuse');

        % Restore the original warning state immediately.
        warning(warnState); 
        
        % Explicitly make the figure visible and bring it to the front.
        figure(hFig);

        % Find all axes objects within the figure.
        hAxes = findall(hFig, 'type', 'axes');

        if isempty(hAxes)
            fprintf('     -> No axes found in this figure. Skipping.\n');
            close(hFig);
            continue;
        end

        % Loop through each axes object (subplot or main plot).
        for j = 1:length(hAxes)
            currentAxes = hAxes(j);

            % ----- FONT SIZE MODIFICATION -----
            % Set font size for the tick labels (the numbers on the axes).
            set(currentAxes, 'FontSize', fontSize.tickLabels);
            
            % Set font size for the X, Y, and Z axis labels.
            set(currentAxes.XLabel, 'FontSize', fontSize.axisLabels);
            set(currentAxes.YLabel, 'FontSize', fontSize.axisLabels);
            set(currentAxes.ZLabel, 'FontSize', fontSize.axisLabels);
            
            % Set font size for the title.
            set(currentAxes.Title, 'FontSize', fontSize.title);
            
            % Get the legend object directly from the axes' 'Legend' property.
            hLegend = currentAxes.Legend;
            if ~isempty(hLegend) && isvalid(hLegend)
                set(hLegend, 'FontSize', fontSize.legend);
            end
        end % end of axes loop
        
        % MODIFICATION: Force MATLAB to render the changes and update message.
        drawnow;
        fprintf('     -> Figure modified and left open for inspection.\n');
        
        % MODIFICATION: Saving and closing have been commented out.
        % To re-enable, uncomment the following lines.
        % fprintf('     -> Saving changes.\n');
        % savefig(hFig, filePath);
        % close(hFig);

    catch ME
        warning(warnState); % Ensure warning state is restored on error.
        fprintf('     -> An error occurred while processing "%s":\n', filePath);
        fprintf('        %s\n', ME.message);
        % Close all figures in case of an error to prevent memory leaks.
        close all;
    end
end

fprintf('\nFont resizing complete.\n');
fprintf('Warning: All modified figures have been left open.\n');
fprintf('Please review, save, and close them manually.\n');

