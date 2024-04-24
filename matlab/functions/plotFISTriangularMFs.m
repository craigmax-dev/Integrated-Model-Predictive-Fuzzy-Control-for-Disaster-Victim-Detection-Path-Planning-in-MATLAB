function plotFISTriangularMFs(fis)
    % Number of inputs in the FIS
    numInputs = numel(fis.Inputs);

    % Create a figure for the plots
    figure;

    % Loop over each input to create subplots
    for i = 1:numInputs
        subplot(numInputs, 1, i);
        hold on;
        
        % Format the input name to display subscript for text following "_"
        formattedInputName = formatInputName(fis.Inputs(i).Name);
        title(formattedInputName);

        xlabel("Input Value");
        ylabel("Membership Degree");

        % Number of MFs for the current input
        numMFs = numel(fis.Inputs(i).MembershipFunctions);

        % Plot each MF
        for j = 1:numMFs
            mf = fis.Inputs(i).MembershipFunctions(j);
            % Assuming the MF type is "trimf" for triangular
            if strcmp(mf.Type, "trimf")
                % Extract the parameters [a, b, c] of the triangular MF
                params = mf.Parameters;
                % Define the x values (range from a to c)
                x = linspace(params(1), params(3), 100);
                % Triangular membership function calculation
                y = max(min((x-params(1))/(params(2)-params(1)), (params(3)-x)/(params(3)-params(2))), 0);
                plot(x, y, "DisplayName", mf.Name);
            end
        end
        
        % Add legend to the subplot
        legend show;
        hold off;
    end

    % Adjust subplot layouts to avoid overlap
    sgtitle("Triangular Membership Functions of FIS Inputs");
    set(gcf, "Name", "FIS Membership Functions", "NumberTitle", "off");
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