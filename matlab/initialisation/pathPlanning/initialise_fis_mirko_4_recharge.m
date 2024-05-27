%% Function initialise_pathPlanning
% Initialise path planning model

% NOTE: In progress, battery model not properly implemented in FIS

% V2.3 - battery recharge model
function [fisArray] = initialise_fis_mirko_4_recharge(n_a)
    inputs = ["t_response", "cell_priority", "cell_scan_certainty", "cell_fire_time_risk", "battery_level", "distance_to_recharge"];
    inputRanges = [0 1; 0 1; 0 1; 0 100; 0 1; 0 1]; % Updated ranges to include battery level and distance to recharge
    mfTypes = ["trimf", "trimf", "trimf", "trimf", "trimf", "trimf"]; % Include MF types for new inputs
    mfNames = ["low", "medium", "high"]; % Names for MFs remain the same, apply to new inputs as well
    inputPreferences = ["minimize", "maximize", "minimize", "minimize", "maximize", "minimize"]; % Include preferences for battery level and distance

    % Initialize the basic FIS for each agent
    fisArray = arrayfun(@(x) initSingleFIS(inputs(1:4), inputRanges(1:4, :), mfTypes(1:4), mfNames, inputPreferences(1:4)), 1:n_a);
    
    % Modify each FIS to add battery recharge logic
    for idx = 1:n_a
        % Add new inputs for battery level and distance to recharge point
        fisArray(idx) = addInput(fisArray(idx), inputRanges(5, :), 'Name', 'battery_level');
        fisArray(idx) = addInput(fisArray(idx), inputRanges(6, :), 'Name', 'distance_to_recharge');
        
        % Add MFs for these new inputs
        for mfIndex = 1:length(mfNames)
            fisArray(idx) = addMF(fisArray(idx), 'battery_level', mfTypes(5), calculateMFParams(inputRanges(5, :), mfIndex, length(mfNames)), 'Name', mfNames(mfIndex));
            fisArray(idx) = addMF(fisArray(idx), 'distance_to_recharge', mfTypes(6), calculateMFParams(inputRanges(6, :), mfIndex, length(mfNames)), 'Name', mfNames(mfIndex));
        end
    end
end

function params = calculateMFParams(inputRange, mfIndex, totalMFs)
    % Helper function to calculate parameters for triangular membership functions
    step = (inputRange(2) - inputRange(1)) / (totalMFs - 1);
    params = [max(inputRange(1), inputRange(1) + step * (mfIndex - 2)), ...
              inputRange(1) + step * (mfIndex - 1), ...
              min(inputRange(2), inputRange(1) + step * mfIndex)];
end
