% This script opens every image file nested within a specified folder,
% then iterates through each figure and its subplots to modify text
% elements, font sizes, and other properties.
%
% MODIFIED: After processing all figures, it pauses and provides an
% interactive prompt to save all open figures as both .fig and .svg.

% ----- USER INPUTS -----
% Specify the folder containing the images.
folderPath = 'C:\Users\CMAX2647\Documents\GitHub\Integrated-Model-Predictive-Fuzzy-Control-for-Disaster-Victim-Detection-Path-Planning-in-MATLAB\simulations\4_2_5_two_agent_complex_dynamic_env_params_v2';
% folderPath = 'C:\Users\CMAX2647\Documents\GitHub\Integrated-Model-Predictive-Fuzzy-Control-for-Disaster-Victim-Detection-Path-Planning-in-MATLAB\simulations\4_3_sensitivity_analysis_plots';
% folderPath = 'C:\Users\CMAX2647\Documents\GitHub\Integrated-Model-Predictive-Fuzzy-Control-for-Disaster-Victim-Detection-Path-Planning-in-MATLAB\simulations\plots_environment_and_agents';

% Define the font sizes for different text elements.
fontSize.axisLabels = 18;
fontSize.tickLabels = 16;
fontSize.title = 18;
fontSize.legend = 18;

% Define the text replacement rules as an N-by-2 cell array.
replacementRules = {
    'Optimisation time, $\overline{t}^{\mathrm{opt}}\mathrm{(s)}$', 'Optimisation time, $\overline{t}^{\mathrm{opt}}$(k)[s]';
    'Optimisation time, $\overline{t}^{\mathrm{opt}}$ (s)', 'Optimisation time, $\overline{t}^{\mathrm{opt}}$(k)[s]';
    'Number of robots, $n^{r}$', 'Number of robots, $n^{\mathrm{rob}}$';
    '\mathbf{M}', 'M';
    'm/s', '\mathrm{m/s}';
    '$T^{\mbox{risk}}$', '$M^{\mbox{risk}}$';
%     'Parameter Index', 'Parameter index';
%     'Membership Function', 'Membership function';
%     'FIS Output MF Parameters', '';
%     '$T^{\mbox{risk}}$', '$M^{\mbox{risk}}$';
%     'MPFC', 'M2PFC';
%     'FIS', 'FLC';
%     'MF', 'Membership Function';
%     'T^{risk}', 'M^{risk}';
%     'Optimisation time, $\overline{t}^{\mathrm{opt}}$', 'Optimisation time, $\overline{t}^{\mathrm{opt}}\mathrm{(s)}$';
%     '$\mathbf{M}^{\mathrm{struct}}(41) = 1_{[20 \times 20]}$', '$M^{\mathrm{struct}}(41) = 1_{[20 \times 20]}$';
%     '$\mathbf{M}^{\mathrm{struct}}(21) = 1_{[20 \times 20]}$', '$M^{\mathrm{struct}}(21) = 1_{[20 \times 20]}$';
    '$M^{\mathrm{struct}} = 1_{[20 \times 20]}$', '$M^{\mathrm{struct}}(1) = 1_{[20 \times 20]}$';
%     '$\mathbf{M}^{\mathrm{struct}}(41) = 0.5_{[20 \times 20]}$', '$M^{\mathrm{struct}}(41) = 0.5_{[20 \times 20]}$';
%     '$\mathbf{M}^{\mathrm{struct}}(21) = 0.5_{[20 \times 20]}$', '$M^{\mathrm{struct}}(21) = 0.5_{[20 \times 20]}$';
%     '$\mathbf{M}^{\mathrm{struct}}(1) = 0.5_{[20 \times 20]}$', '$M^{\mathrm{struct}}(1) = 0.5_{[20 \times 20]}$';
%     '$\mathbf{M}^{\mathrm{struct}}(41) = 0.25_{[20 \times 20]}$', '$M^{\mathrm{struct}}(41) = 0.25_{[20 \times 20]}$';
%     '$\mathbf{M}^{\mathrm{struct}}(21) = 0.25_{[20 \times 20]}$', '$M^{\mathrm{struct}}(21) = 0.25_{[20 \times 20]}$';
%     '$\mathbf{M}^{\mathrm{struct}}(1) = 0.25_{[20 \times 20]}$', '$M^{\mathrm{struct}}(1) = 0.25_{[20 \times 20]}$';
%     '$\mathbf{M}^{\mathrm{victim}}(1)$';
%     '$\mathbf{M}^{\mathrm{struct}}$';
%     '$\mathbf{M}^{\mathrm{debris}}(1)$';
%     '$\mathbf{M}^{\mathrm{debris,coarsened}}(1)$';
%     '$\mathbf{F}$, $v^{\mathrm{wind}} = 0$ m/s', '$\Pi$, $m^{\mathrm{velocity}} = 0$ m/s';
%     '$\mathbf{F}$, $v^{\mathrm{wind}} = 1$ m/s', '$\Pi$, $m^{\mathrm{velocity}} = 1$ m/s';
%     '$\mathbf{F}$, $v^{\mathrm{wind}} = 3$ m/s', '$\Pi$, $m^{\mathrm{velocity}} = 3$ m/s';
%     '$t^{\mathrm{travel}}$, $v^{\mathrm{wind}} = 3$ m/s', '$t^{\mathrm{travel}}_{r}$, $m^{\mathrm{velocity}} = 3$ m/s';
%     '$t^{\mathrm{travel}}$, $v^{\mathrm{wind}} = 1$ m/s', '$t^{\mathrm{travel}}_{r}$, $m^{\mathrm{velocity}} = 1$ m/s';
%     '$t^{\mathrm{travel}}$, $v^{\mathrm{wind}} = 0$ m/s', '$t^{\mathrm{travel}}_{r}$, $m^{\mathrm{velocity}} = 0$ m/s';
%     '$t^{\mbox{response}}$', '$M^{\mbox{response}}$';
%     '$m^{\mbox{priority}}$', '$M^{\mbox{victim}}$';
%     '$m^{\mbox{scan}}$', '$M^{\mbox{scan}}$';
%     '$t^{\mbox{fire risk}}$', '$T^{\mbox{risk}}$';
%     '$k = 41$', '$t = 2460 \mathrm{s}$';
%     '$k = 21$', '$t = 1260 \mathrm{s}$';
%     '$k = 1$', '$t = 60 \mathrm{s}$';
%     '$k = 30$', '$t = 1800 \mathrm{s}$';
%     '$k = 60$', '$t = 3600 \mathrm{s}$';
%     '$k = 90$', '$t = 5400 \mathrm{s}$';
%     'Number of Agents, $n^{a}$', 'Number of Robots, $n^{r}$';
%     'Prediction Horizon, $\Delta t^{\mathrm{pred}}\mathrm{(s)}$', 'Prediction Step, $N^{\mathrm{P}}$';
%     'Simulation Time, $t$ (s)', 'Simulation Timestep, $k$';
%     'MPC Timestep, $\Delta t^{\mathrm{MPC}}$(s)', '$T^{\mathrm{ctrl}}$';
%     'Mean', 'mean';    
};


% ----- SCRIPT LOGIC (PART 1: MODIFY FIGURES) -----

if ~isfolder(folderPath)
    error('Error: The specified folder "%s" does not exist.', folderPath);
end

imageFiles = dir(fullfile(folderPath, '**', '*.fig')); 

if isempty(imageFiles)
    warning('No .fig files found in the specified folder "%s".', folderPath);
    return;
end

fprintf('Processing %d image files...\n', length(imageFiles));

for i = 1:length(imageFiles)
    filePath = fullfile(imageFiles(i).folder, imageFiles(i).name);
    fprintf('  -> Opening and modifying: %s\n', filePath);

    warnState = warning('off', 'MATLAB:load:classNotFound');
    try
        hFig = openfig(filePath, 'reuse');
        warning(warnState); 
        figure(hFig);

        hAxes = findall(hFig, 'type', 'axes');

        if isempty(hAxes)
            fprintf('     -> No axes found. Skipping.\n');
            close(hFig);
            continue;
        end

        for j = 1:length(hAxes)
            currentAxes = hAxes(j);

            % Font size modification
            set(currentAxes, 'FontSize', fontSize.tickLabels);
            set(currentAxes.XLabel, 'FontSize', fontSize.axisLabels);
            set(currentAxes.YLabel, 'FontSize', fontSize.axisLabels);
            set(currentAxes.ZLabel, 'FontSize', fontSize.axisLabels);
            set(currentAxes.Title, 'FontSize', fontSize.title);
            hLegend = currentAxes.Legend;
            if ~isempty(hLegend) && isvalid(hLegend)
                set(hLegend, 'FontSize', fontSize.legend);
            end

            % Text replacement logic
            textObjects = {currentAxes.Title, currentAxes.XLabel, currentAxes.YLabel, currentAxes.ZLabel};
            if ~isempty(hLegend) && isvalid(hLegend), textObjects{end+1} = hLegend; end
            otherText = findall(currentAxes, 'Type', 'text', '-not', {'Handle', [textObjects{:}]});
            if ~isempty(otherText), textObjects = [textObjects, num2cell(otherText)']; end

            for k = 1:length(textObjects)
                obj = textObjects{k};
                if ~isvalid(obj) || isempty(obj.String), continue; end
                
                if isa(obj, 'matlab.graphics.illustration.Legend')
                    originalStrings = obj.String;
                    newStrings = originalStrings;
                    plotChildren = obj.PlotChildren;
                    numEntries = min(length(originalStrings), length(plotChildren));

                    for entryIdx = 1:numEntries
                        if strcmp(get(plotChildren(entryIdx), 'HandleVisibility'), 'on')
                            stringToModify = newStrings{entryIdx};
                            for ruleIdx = 1:size(replacementRules, 1)
                                stringToModify = replace(stringToModify, replacementRules{ruleIdx, 1}, replacementRules{ruleIdx, 2});
                            end
                            newStrings{entryIdx} = stringToModify;
                        end
                    end
                    obj.String = newStrings;
                else
                    modifiedString = obj.String;
                    for ruleIdx = 1:size(replacementRules, 1)
                        modifiedString = replace(modifiedString, replacementRules{ruleIdx, 1}, replacementRules{ruleIdx, 2});
                    end
                    obj.String = modifiedString;
                end
            end
        end
        
        drawnow;
        fprintf('     -> Figure modified and left open for inspection.\n');

    catch ME
        warning(warnState); 
        fprintf('     -> AN ERROR OCCURRED while processing "%s":\n', filePath);
        fprintf('        %s\n', ME.message);
        close all;
    end
end

fprintf('\n--- All figures have been processed and are open for review. ---\n');


% ----- SCRIPT LOGIC (PART 2: INTERACTIVE SAVE) -----

% Ask the user if they want to save the figures.
prompt = 'Do you want to save all modified figures? (y/n): ';
userInput = input(prompt, 's');

if strcmpi(userInput, 'y')
    % Ask for a suffix.
    suffixPrompt = 'Enter a filename suffix (e.g., _v2) or press Enter to overwrite: ';
    fileSuffix = input(suffixPrompt, 's');
    
    % Get handles to all open figures.
    allHandles = findall(0, 'Type', 'figure');
    if isempty(allHandles)
        disp('No open figures found to save.');
        return;
    end
    
    fprintf('\nStarting save process for %d figure(s)...\n', length(allHandles));
    
    for hFig = allHandles'
        originalFilepath = get(hFig, 'FileName');
        
        if isempty(originalFilepath)
            fprintf(' -> Skipping Figure %d (was not opened from a file).\n', hFig.Number);
            continue;
        end
        
        [folder, name, ~] = fileparts(originalFilepath);
        
        % Construct the new filenames with the suffix.
        newBaseName = [name, fileSuffix];
        newFigPath = fullfile(folder, [newBaseName, '.fig']);
        newSvgPath = fullfile(folder, [newBaseName, '.svg']);
        
        fprintf(' -> Saving Figure %d ("%s")\n', hFig.Number, newBaseName);
        
        try
            % Save as .fig
            savefig(hFig, newFigPath);
            fprintf('    ... saved as %s\n', [newBaseName, '.fig']);
            
            % Save as .svg using the compatible 'print' function
            print(hFig, newSvgPath, '-dsvg');
            fprintf('    ... saved as %s\n', [newBaseName, '.svg']);
            
            % Close the figure after saving.
            close(hFig);
            
        catch ME_save
            fprintf('    ... FAILED to save Figure %d: %s\n', hFig.Number, ME_save.message);
        end
    end
    
    fprintf('\nSave and close process complete.\n');
    
else
    fprintf('\nSave operation cancelled. Figures remain open for manual review.\n');
end