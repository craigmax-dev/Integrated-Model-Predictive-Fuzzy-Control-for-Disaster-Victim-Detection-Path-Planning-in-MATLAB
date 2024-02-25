%% Function initialise_pathPlanning
% Initialise path planning model

% V2
% CHANGELOG
% - refactor: correct rules, add linear output functions
% - refactor: removed t_dw as this is considered in priority calculation instead

% V2.3
function [fisArray] = initialise_fis_t_response_priority(n_a)
    inputs = ["t_response", "priority"];
    inputRanges = [0 1; 0 1000]; % [0, 1] for t_response, [0, 1000] for priority
    mfTypes = ["trimf", "trimf"]; % Triangular MFs for both inputs
    mfNames = ["low", "medium", "high"]; % Names for MFs
    inputPreferences = ["minimize", "maximize"];
    
    fisArray = arrayfun(@(x) initSingleFIS(inputs, inputRanges, mfTypes, mfNames, inputPreferences), 1:n_a);
end

