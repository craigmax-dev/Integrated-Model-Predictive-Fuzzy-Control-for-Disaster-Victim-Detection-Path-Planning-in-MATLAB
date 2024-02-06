function saveSimulationResults(flag, data, filename)
    % Check if the flag is true
    if flag
        % Save the data to a file
        save(filename, 'data');
        fprintf('Simulation results saved to %s\n', filename);

        % Find all open figures
        figHandles = findobj('Type', 'figure');

        % Save each figure
        for i = 1:length(figHandles)
            figName = sprintf('%s_fig%d', filename, i);
            savefig(figHandles(i), figName);
            print(figHandles(i), figName, '-dpng'); % Save as PNG
            fprintf('Figure %d saved as %s.fig and %s.png\n', i, figName, figName);
        end
    else
        % Do not save, and output a message indicating this
        fprintf('Saving of simulation results is not enabled.\n');
    end
end
