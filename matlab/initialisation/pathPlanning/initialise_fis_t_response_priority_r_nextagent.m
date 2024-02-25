%% Function initialise_pathPlanning
% Initialise path planning model

% V2
% CHANGELOG
% - refactor: correct rules, add linear output functions
% - refactor: removed t_dw as this is considered in priority calculation instead

% V2.3
function [fisArray] = initialise_fis_t_response_priority_r_nextagent(n_a)
    inputs = ["t_response", "priority", "r_nextagent"];
    inputPreferences = ["minimize", "maximize", "maximize"];
    inputRanges = [0 1; 0 1000; 0 1];
    mfTypes = ["trimf", "trimf", "trimf"]; % MF type for each inputs
    mfNames = ["low", "medium", "high"]; % Number and names for MF of each input
    
    fisArray = arrayfun(@(x) initSingleFIS(inputs, inputRanges, mfTypes, mfNames, inputPreferences), 1:n_a);
end