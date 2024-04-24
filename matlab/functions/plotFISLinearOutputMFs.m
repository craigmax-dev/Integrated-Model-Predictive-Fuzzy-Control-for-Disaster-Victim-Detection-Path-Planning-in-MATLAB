
% V1
% Create a surface plot given two inputs MAX

function plotFISLinearOutputMFs(fis)
    % Assuming the FIS has one output and the inputs are all the same range.
    % Generate a grid for the inputs (assuming two inputs for demonstration purposes)
    [inputGridX, inputGridY] = meshgrid(linspace(0, 1, 100), linspace(0, 1, 100));

    % Flatten the grids for evaluation
    inputGridXFlat = inputGridX(:);
    inputGridYFlat = inputGridY(:);

    % Create a figure for the plot
    figure;

    % Number of MFs for the output
    numMFs = numel(fis.Outputs(1).MembershipFunctions);

    % Plot each linear MF
    for i = 1:numMFs
        % Get the MF and its parameters
        mf = fis.Outputs(1).MembershipFunctions(i);
        params = mf.Parameters;

        % Calculate the output of the linear function over the input grid
        % Assuming a function of form: y = a*x1 + b*x2 + c
        outputGrid = params(1) * inputGridXFlat + params(2) * inputGridYFlat + params(end);

        % Reshape back to a grid for plotting
        outputGrid = reshape(outputGrid, size(inputGridX));

        % Plot
        surf(inputGridX, inputGridY, outputGrid, 'DisplayName', mf.Name);
        hold on;
    end

    % Customize plot
    title('Linear Membership Functions of FIS Output');

    formattedInputName = formatInputName(fis.Inputs(1).Name);
    xlabel(formattedInputName);
    
    formattedInputName = formatInputName(fis.Inputs(2).Name);
    ylabel(formattedInputName);
    
    formattedOutputName = formatInputName(fis.Outputs(1).Name);
    ylabel(formattedOutputName);

    legend show;
    hold off;

    set(gcf, 'Name', 'FIS Linear Output Membership Functions', 'NumberTitle', 'off');
end

% Utility function to format input names for plot titles
function formattedName = formatInputName(inputName)
    parts = split(inputName, '_');
    if numel(parts) > 1
        % Concatenate parts with underscore and braces for subscript
        formattedName = strcat(parts{1}, '_{', strjoin(parts(2:end), ' '), '}');
    else
        formattedName = inputName;
    end
    % Remove any newline characters that may exist
    formattedName = strrep(formattedName, newline, '');
    % Remove any leading or trailing whitespace
    formattedName = strtrim(formattedName);
end