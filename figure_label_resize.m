% batch_resize_and_export.m
%
% This script automates the process of resizing fonts for all figures under
% a specified directory and exporting them to SVG format.
%
% MODIFIED: Uses the compatible 'print' function instead of 'exportgraphics'
% to support MATLAB versions older than R2020a.

% ----- USER INPUTS -----
% Specify the top-level folder containing your .fig files.
% Use a full path, e.g., 'C:\Users\YourName\Documents\MyImages'
folderPath = 'C:\Users\CMAX2647\Documents\GitHub\Integrated-Model-Predictive-Fuzzy-Control-for-Disaster-Victim-Detection-Path-Planning-in-MATLAB\simulations';

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

% Get a list of all .fig files in the folder and all its subfolders.
imageFiles = dir(fullfile(folderPath, '**', '*.fig')); 

% Check if any image files were found.
if isempty(imageFiles)
    warning('No .fig files found in the specified folder "%s".', folderPath);
    return;
end

% Loop through each found image file.
fprintf('Found %d image files. Starting batch processing...\n', length(imageFiles));

for i = 1:length(imageFiles)
    originalFileName = imageFiles(i).name;
    originalFilePath = fullfile(imageFiles(i).folder, originalFileName);

    fprintf('  -> Processing: %s\n', originalFilePath);

    % Suppress the harmless 'classNotFound' warning.
    warnState = warning('off', 'MATLAB:load:classNotFound');
    try
        % Open the figure invisibly for faster processing.
        hFig = openfig(originalFilePath, 'reuse', 'invisible');

        % Restore the original warning state.
        warning(warnState); 

        hAxes = findall(hFig, 'type', 'axes');

        if isempty(hAxes)
            fprintf('     -> No axes found in this figure. Skipping.\n');
            close(hFig);
            continue;
        end

        for j = 1:length(hAxes)
            currentAxes = hAxes(j);

            % ----- FONT SIZE MODIFICATION -----
            set(currentAxes, 'FontSize', fontSize.tickLabels);
            set(currentAxes.XLabel, 'FontSize', fontSize.axisLabels);
            set(currentAxes.YLabel, 'FontSize', fontSize.axisLabels);
            set(currentAxes.ZLabel, 'FontSize', fontSize.axisLabels);
            set(currentAxes.Title, 'FontSize', fontSize.title);
            
            hLegend = currentAxes.Legend;
            if ~isempty(hLegend) && isvalid(hLegend)
                set(hLegend, 'FontSize', fontSize.legend);
            end
        end % end of axes loop
        
        % ----- SAVE LOGIC -----
        [folder, name, ~] = fileparts(originalFilePath);
        newBaseName = [name, '_1'];
        newSvgPath = fullfile(folder, [newBaseName, '.svg']);
        
        fprintf('     -> Saving as: %s\n', [newBaseName, '.svg']);
        
        % --- MODIFICATION ---
        % Replaced modern 'exportgraphics' with the classic 'print' function,
        % which is compatible with older MATLAB versions.
        % The '-dsvg' flag specifies that the output format should be SVG.
        print(hFig, newSvgPath, '-dsvg');
        % --------------------
        
        close(hFig);

    catch ME
        warning(warnState); 
        fprintf('     -> An error occurred while processing "%s":\n', originalFilePath);
        fprintf('        %s\n', ME.message);
        close all;
    end
end

fprintf('\nBatch processing complete.\n');