% Function to plot membership functions of FIS controllers.

function plotFisMembershipFunctions(fisArray)
    % Number of agents
    numAgents = length(fisArray);

    % Iterate over each agent's FIS
    for a = 1:numAgents
        fis = fisArray(a);

        % Number of input variables
        numInputs = length(fis.Inputs);

        % Create a figure for each agent
        figure;
        sgtitle(['Agent ', num2str(a), ' FIS Input Membership Functions']);

        % Plot each input's membership functions
        for i = 1:numInputs
            subplot(numInputs, 1, i);
            plotmf(fis, 'input', i);
            title(['Input ', num2str(i), ' (', fis.Inputs(i).Name, ')']);
        end
    end
end
