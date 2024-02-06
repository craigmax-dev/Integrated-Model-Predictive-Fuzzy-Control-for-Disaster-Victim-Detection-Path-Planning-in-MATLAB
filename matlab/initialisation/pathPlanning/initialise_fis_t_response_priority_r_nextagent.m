%% Function initialise_pathPlanning
% Initialise path planning model

% V2
% CHANGELOG
% - refactor: correct rules, add linear output functions
% - refactor: removed t_dw as this is considered in priority calculation instead

% TODO
% Check range of priority
% Research possible functions to apply to inputs to normalise within given range
% - e.g. S-shape

%% V2 refactor
function [fisArray] = initialise_fis_t_response_priority_r_nextagent(n_a)

  inputs = ["t_response", "priority", "r_nextagent"];
  inputRanges = [0 1; 0 1000; 0 1];  % [0, 1] for t_response, [0, 1000] for priority

  % Define Membership Function (MF) types and evenly distributed parameters
  mfTypes = ["trimf", "trimf"];  % Triangular MFs for both inputs
  mfNames = ["low", "medium", "high"];  % Names for MFs

  %% Generate FIS
  for a = 1:n_a

    fis = sugfis;

    % Evenly distribute MF parameters for each input
    for i = 1:numel(inputs)
        fis = addInput(fis, inputRanges(i, :), 'Name', inputs(i));
        range = inputRanges(i, :);
        step = (range(2) - range(1)) / 2;
        
        % Create parameters for 'low', 'medium', 'high' MFs
        lowParams = [range(1) - step, range(1), range(1) + step];
        mediumParams = [range(1), (range(1) + range(2)) / 2, range(2)];
        highParams = [range(2) - step, range(2), range(2) + step];
        
        mfParams = [lowParams; mediumParams; highParams];
        
        for j = 1:numel(mfNames)
            fis = addMF(fis, inputs(i), mfTypes(i), mfParams(j, :), 'Name', mfNames(j));
        end
    end

    outputs = "attraction";
    fis = addOutput(fis, [0 1], 'Name', outputs);

    % Output MFs for low, medium, high
    fis = addMF(fis, outputs, 'linear', [0 0 0], 'Name', 'low');    % Low output
    fis = addMF(fis, outputs, 'linear', [0 0 0.5], 'Name', 'medium');  % Medium output
    fis = addMF(fis, outputs, 'linear', [0 0 1], 'Name', 'high');   % High output

    % Dynamically generate ruleList based on inputs
    numMFs = numel(mfNames);
    numInputs = numel(inputs);
    ruleList = [];
    for i = 1:numMFs^numInputs
        inputCombination = dec2base(i-1, numMFs, numInputs) - '0' + 1;
        outputLevel = round(mean(inputCombination));
        rule = [inputCombination, outputLevel, 1, 1];
        ruleList = [ruleList; rule];
    end

    fis = addRule(fis, ruleList);

    %% Add to FIS array
    fisArray(a) = fis;
  end
end

