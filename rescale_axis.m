% SCRIPT to rescale X-axis data of the currently active plot.

ax = gca; 
set(ax.XLabel, 'String', 'Simulation timestep, $k$', 'Interpreter', 'latex');
set(ax.YLabel, 'String', 'Optimisation time, $\overline{t}^{\mathrm{opt}}\mathrm{(s)}$', 'Interpreter', 'latex');

ax = gca; 
set(ax.XLabel, 'String', 'Simulation timestep, $k$', 'Interpreter', 'latex');
set(ax.YLabel, 'String', 'Objective function, $\overline{J}$', 'Interpreter', 'latex');

ax = gca; 
set(ax.XLabel, 'String', 'Number of environment cells, $n^{env^{x}} \cdot n^{env^{y}}$', 'Interpreter', 'latex');
set(ax.YLabel, 'String', 'Normalised objective function, $\overline{J} (\Delta \%)$', 'Interpreter', 'latex');

% ----- USER INPUT -----
% Define the value to divide the x-axis data by.
% For example, to convert seconds to minutes, use 60.
scalingFactor = 1/15;
% ----------------------

% Get the handle to the currently active axes
ax = gca;
fprintf('Targeting the current axes in Figure %d.\n', get(ax, 'Parent').Number);

% Find all children of the axes that have an 'XData' property 
% (e.g., lines, scatter plots, bars, etc.)
dataObjects = findall(ax, '-property', 'XData');

if isempty(dataObjects)
    warning('No plottable data objects (lines, scatter plots, etc.) with XData found in the current axes.');
    return;
end

% Initialize variables to track the new overall x-axis limits
minX = inf;
maxX = -inf;

fprintf('Found %d data object(s) to rescale.\n', length(dataObjects));

% Loop through each plottable object
for i = 1:length(dataObjects)
    obj = dataObjects(i);
    
    % Get the original XData from the object
    originalXData = get(obj, 'XData');
    
    % Rescale the data by dividing by the scaling factor
    newXData = originalXData / scalingFactor;
    
    % Update the object with its new, rescaled XData
    set(obj, 'XData', newXData);
    
    % Update the overall min and max values encountered so far
    % The (:) ensures data is treated as a single vector, handling matrices
    minX = min(minX, min(newXData(:)));
    maxX = max(maxX, max(newXData(:)));
end

% Check if any valid data was found before setting limits
if isfinite(minX) && isfinite(maxX)
    % Add a small padding to the limits for better visualization
    padding = (maxX - minX) * 0; % 5% padding
    padding = (maxX - minX) * 0.02; % 5% padding    
    if padding == 0 % Handle case where minX equals maxX
        padding = 0; 
    end
    
    % Apply the new, padded limits to the x-axis
    set(ax, 'XLim', [minX - padding, maxX + padding]);
    
    disp('X-axis data and limits have been successfully rescaled.');
else
    disp('No finite data was found to rescale, axis limits remain unchanged.');
end

clear ax scalingFactor dataObjects minX maxX padding i obj originalXData newXData;