%% Function initialise_pathPlanning
% Initialise path planning model

% Input range documentation
% "t_response" - [0 1]
% "cell_priority" - [0 1]
% "cell_scan_certainty" - [0 1]
% "priority_dw" - [0 1]
% 'priority_first_scan'
% 'priority_combined'
% 'r_nextagent'

% V2.3
function [fisArray] = initialise_fis_mod(n_a)
    inputs = ["t_response", "cell_priority", "cell_scan_certainty", "priority_dw"];
    inputRanges = [0 1; 0 1; 0 1; 0 1]; % [0, 1] for t_response, [0, 1000] for priority, [0 6 t_fire (high value for no risk)]
    mfTypes = ["trimf", "trimf", "trimf", "trimf"]; % Triangular MFs for both inputs
    mfNames = ["low", "medium", "high"]; % Names for MFs
    inputPreferences = ["minimize", "maximize", "maximize", "minimize"];
    
    fisArray = arrayfun(@(x) initSingleFIS(inputs, inputRanges, mfTypes, mfNames, inputPreferences), 1:n_a);
end