% Function to plot membership functions of FIS controllers.

% inputNamesLatex = ["$t^{\mbox{response}}$", "$m^{\mbox{priority}}$", "$m^{\mbox{scan}}$", "$t^{\mbox{fire risk}}$"];

function plotFisMembershipFunctions(fis, inputNamesLatex)
    % Validate the number of provided LaTeX names against the number of inputs
    if length(inputNamesLatex) ~= numel(fis.Inputs)
        error('The number of input LaTeX names must match the number of FIS inputs.');
    end

    % Number of input variables
    numInputs = numel(fis.Inputs);

    % Create a figure for the membership function plots
    figure;
    sgtitle('FIS Input Membership Functions', 'Interpreter', 'latex');

    % Plot each input's membership functions
    for i = 1:numInputs
        subplot(numInputs, 1, i);
        plotmf(fis, 'input', i);
        title(inputNamesLatex{i}, 'Interpreter', 'latex');  % Explicitly set the interpreter for the title
        xlabel('Value', 'Interpreter', 'latex');  % Ensure xlabel uses LaTeX interpreter
        ylabel('Membership', 'Interpreter', 'latex');  % Ensure ylabel uses LaTeX interpreter
    end
end
