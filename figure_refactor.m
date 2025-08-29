% ax = gca; 
% set(ax.XLabel, 'String', 'Simulation Timestep, $k$', 'Interpreter', 'latex');

% This script opens every image file nested within a specified folder,
% then iterates through each figure and its subplots to modify text
% elements (titles, labels, legends, annotations) and apply specified 
% font sizes.
% MODIFIED: Figures will be left open for manual inspection and will not be
% automatically saved.
% MODIFIED: Legend modification now respects hidden entries and will not
% make them visible again.

% ----- USER INPUTS -----
% Specify the folder containing the images.
% Use a full path, e.g., 'C:\Users\YourName\Documents\MyImages'
folderPath = 'C:\Users\CMAX2647\Documents\GitHub\Integrated-Model-Predictive-Fuzzy-Control-for-Disaster-Victim-Detection-Path-Planning-in-MATLAB\simulations\4_2_5_two_agent_complex_dynamic_env_params_v2';

% Define the font sizes for different text elements.
fontSize.axisLabels = 18;
fontSize.tickLabels = 14;
fontSize.title = 18;
fontSize.legend = 18;

% Define the text replacement rules as an N-by-2 cell array.
% The format is { 'old text 1', 'new text 1';
%                'old text 2', 'new text 2'; ... }
% The order is important for rules that might overlap.
replacementRules = {
    'MPFC', 'M2PFC';
    'FIS', 'FLC';
    'MF', 'Membership Function';
    'T^{risk}', 'M^{risk}';
%     'MPC Timestep, $\Delta t^{\mathrm{MPC}}$', 'MPC Timestep, $\Delta t^{\mathrm{MPC}}\mathrm{(s)}$';
%     'Prediction Horizon, $\Delta t^{\mathrm{pred}}$', 'Prediction Horizon, $\Delta t^{\mathrm{pred}}\mathrm{(s)}$';
    'Optimisation time, $\overline{t}^{\mathrm{opt}}$', 'Optimisation time, $\overline{t}^{\mathrm{opt}}\mathrm{(s)}$';
    '$k = 41, \mathbf{M}^{\mathrm{structure}} = 1$', '$\mathbf{M}^{\mathrm{struct}}(41) = 1_{[20 \times 20]}$';
    '$k = 21, \mathbf{M}^{\mathrm{structure}} = 1$', '$\mathbf{M}^{\mathrm{struct}}(21) = 1_{[20 \times 20]}$';
    '$k = 1, \mathbf{M}^{\mathrm{structure}} = 1$', '$\mathbf{M}^{\mathrm{struct}} = 1_{[20 \times 20]}$';
    '$k = 41, \mathbf{M}^{\mathrm{structure}} = 0.5$', '$\mathbf{M}^{\mathrm{struct}}(41) = 0.5_{[20 \times 20]}$';
    '$k = 21, \mathbf{M}^{\mathrm{structure}} = 0.5$', '$\mathbf{M}^{\mathrm{struct}}(21) = 0.5_{[20 \times 20]}$';
    '$k = 1, \mathbf{M}^{\mathrm{structure}} = 0.5$', '$\mathbf{M}^{\mathrm{struct}}(1) = 0.5_{[20 \times 20]}$';
    '$k = 41, \mathbf{M}^{\mathrm{structure}} = 0.25$', '$\mathbf{M}^{\mathrm{struct}}(41) = 0.25_{[20 \times 20]}$';
    '$k = 21, \mathbf{M}^{\mathrm{structure}} = 0.25$', '$\mathbf{M}^{\mathrm{struct}}(21) = 0.25_{[20 \times 20]}$';
    '$k = 1, \mathbf{M}^{\mathrm{structure}} = 0.25$', '$\mathbf{M}^{\mathrm{struct}}(1) = 0.25_{[20 \times 20]}$';
    '$\mathbf{M}^{\mathrm{victim}}$', '$\mathbf{M}^{\mathrm{victim}}(1)$';
    '$\mathbf{M}^{\mathrm{structure}}$', '$\mathbf{M}^{\mathrm{struct}}$';
    '$\mathbf{M}^{\mathrm{building}}$', '$\mathbf{M}^{\mathrm{debris}}(1)$';
    '$\mathbf{M}^{\mathrm{c,building}}$', '$\mathbf{M}^{\mathrm{debris,coarsened}}(1)$';
    '$\mathbf{F}$, $v^{\mathrm{wind}} = 0$ m/s', '$\Pi$, $m^{\mathrm{velocity}} = 0$ m/s';
    '$\mathbf{F}$, $v^{\mathrm{wind}} = 1$ m/s', '$\Pi$, $m^{\mathrm{velocity}} = 1$ m/s';
    '$\mathbf{F}$, $v^{\mathrm{wind}} = 3$ m/s', '$\Pi$, $m^{\mathrm{velocity}} = 3$ m/s';
    '$t^{\mathrm{travel}}$, $v^{\mathrm{wind}} = 3$ m/s', '$t^{\mathrm{travel}}_{r}$, $m^{\mathrm{velocity}} = 3$ m/s';
    '$t^{\mathrm{travel}}$, $v^{\mathrm{wind}} = 1$ m/s', '$t^{\mathrm{travel}}_{r}$, $m^{\mathrm{velocity}} = 1$ m/s';
    '$t^{\mathrm{travel}}$, $v^{\mathrm{wind}} = 0$ m/s', '$t^{\mathrm{travel}}_{r}$, $m^{\mathrm{velocity}} = 0$ m/s';
    '$t^{\mbox{response}}$', '$M^{\mbox{response}}$';
    '$m^{\mbox{priority}}$', '$M^{\mbox{victim}}$';
    '$m^{\mbox{scan}}$', '$M^{\mbox{scan}}$';
    '$t^{\mbox{fire risk}}$', '$T^{\mbox{risk}}$';
    '$k = 41$', '$t = 2460 \mathrm{s}$';
    '$k = 21$', '$t = 1260 \mathrm{s}$';
    '$k = 1$', '$t = 60 \mathrm{s}$';
    '$k = 30$', '$t = 1800 \mathrm{s}$';
    '$k = 60$', '$t = 3600 \mathrm{s}$';
    '$k = 90$', '$t = 5400 \mathrm{s}$';
    'Number of Agents, $n^{a}$', 'Number of Robots, $n^{r}$';
    'Prediction Horizon, $\Delta t^{\mathrm{pred}}\mathrm{(s)}$', 'Prediction Step, $N^{\mathrm{P}}$';
    'Simulation Time, $t$ (s)', 'Simulation Timestep, $k$';
    'MPC Timestep, $\Delta t^{\mathrm{MPC}}$(s)', '$T^{\mathrm{ctrl}}$';
    'Mean', 'mean';    
};

% $k = 41, \mathbf{M}^{\mathrm{structure}} = 1$

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
fprintf('Processing %d image files...\n', length(imageFiles));

for i = 1:length(imageFiles)
    fileName = imageFiles(i).name;
    filePath = fullfile(imageFiles(i).folder, fileName);

    fprintf('  -> Opening and modifying: %s\n', filePath);

    % Suppress the harmless 'classNotFound' warning during loading.
    warnState = warning('off', 'MATLAB:load:classNotFound');
    try
        hFig = openfig(filePath, 'reuse');
        warning(warnState); 
        figure(hFig);

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

            % ----- TEXT REPLACEMENT -----
            textObjectsToModify = {
                currentAxes.Title, ...
                currentAxes.XLabel, ...
                currentAxes.YLabel, ...
                currentAxes.ZLabel
            };
            if ~isempty(hLegend) && isvalid(hLegend)
                textObjectsToModify{end+1} = hLegend;
            end
            otherTextHandles = findall(currentAxes, 'Type', 'text', ...
                '-not', {'Handle', currentAxes.Title}, ...
                '-not', {'Handle', currentAxes.XLabel}, ...
                '-not', {'Handle', currentAxes.YLabel}, ...
                '-not', {'Handle', currentAxes.ZLabel});
            if ~isempty(otherTextHandles)
                textObjectsToModify = [textObjectsToModify, num2cell(otherTextHandles)'];
            end

            % --- MODIFIED SECTION: Intelligent Text Replacement ---
            for k = 1:length(textObjectsToModify)
                obj = textObjectsToModify{k};
                
                if ~isvalid(obj) || isempty(obj.String)
                    continue; % Skip invalid or empty objects
                end

                % Check if the current object is a legend
                isLegend = isa(obj, 'matlab.graphics.illustration.Legend');
                
                if isLegend
                    % --- Special Handling for Legends ---
                    originalStrings = obj.String;
                    newStrings = originalStrings; % Start with a copy
                    plotChildren = obj.PlotChildren;
                    
                    numEntries = min(length(originalStrings), length(plotChildren));

                    for entryIdx = 1:numEntries
                        % Check if the corresponding plot item is intended to be visible
                        if strcmp(get(plotChildren(entryIdx), 'HandleVisibility'), 'on')
                            
                            % This entry is VISIBLE, so apply replacements to its string
                            stringToModify = newStrings{entryIdx};
                            for ruleIdx = 1:size(replacementRules, 1)
                                oldText = replacementRules{ruleIdx, 1};
                                newText = replacementRules{ruleIdx, 2};
                                stringToModify = replace(stringToModify, oldText, newText);
                            end
                            newStrings{entryIdx} = stringToModify;
                        end
                        % If HandleVisibility is 'off', we do nothing, preserving the original string.
                    end
                    obj.String = newStrings;

                else
                    % --- Standard Handling for other text objects (Title, Labels, etc.) ---
                    modifiedString = obj.String;
                    for ruleIdx = 1:size(replacementRules, 1)
                        oldText = replacementRules{ruleIdx, 1};
                        newText = replacementRules{ruleIdx, 2};
                        modifiedString = replace(modifiedString, oldText, newText);
                    end
                    obj.String = modifiedString;
                end
            end
            % --- End of Modified Section ---

        end % end of axes loop
        
        drawnow;
        fprintf('     -> Figure modified and left open for inspection.\n');

    catch ME
        warning(warnState); 
        fprintf('     -> An error occurred while processing "%s":\n', filePath);
        fprintf('        %s\n', ME.message);
        close all;
    end
end

fprintf('\nScript execution complete.\n');
fprintf('Warning: All modified figures have been left open.\n');
fprintf('Please review, save, and close them manually.\n');