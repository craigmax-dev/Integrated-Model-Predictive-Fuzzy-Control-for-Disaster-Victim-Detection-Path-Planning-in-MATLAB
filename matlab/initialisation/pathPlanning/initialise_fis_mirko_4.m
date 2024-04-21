%% Function initialise_pathPlanning
% Initialise path planning model

% V2.3
function [fisArray] = initialise_fis_mirko_4(n_a)
    inputs = ["t_response", "cell_priority", "cell_scan_certainty", "cell_fire_time_risk"];
    inputRanges = [0 1; 0 1; 0 1; 0 100]; % [0, 1] for t_response, [0, 1000] for priority, [0 6 t_fire (high value for no risk)]
    mfTypes = ["trimf", "trimf", "trimf", "trimf"]; % Triangular MFs for both inputs
    mfNames = ["low", "medium", "high"]; % Names for MFs
    inputPreferences = ["minimize", "maximize", "minimize", "minimize"];
    
    fisArray = arrayfun(@(x) initSingleFIS(inputs, inputRanges, mfTypes, mfNames, inputPreferences), 1:n_a);
end