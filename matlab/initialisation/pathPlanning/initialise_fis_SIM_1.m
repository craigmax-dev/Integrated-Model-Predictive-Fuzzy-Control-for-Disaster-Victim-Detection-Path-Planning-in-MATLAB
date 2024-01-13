%% Function initialise_pathPlanning
% Initialise path planning model

% V2
% CHANGELOG
% - refactor: correct rules, add linear output functions
% - refactor: removed t_dw as this is considered in priority calculation instead

%% V2 refactor
function [fisArray] = initialise_fis_SIM_1(n_a)

    inputs = ["t_response", "priority"];
    mfTypes = ["trimf", "trimf"];  % Using triangular MFs for simplicity
    mfNames = ["low", "medium", "high"];
    mfParams = [-0.1 0.5 1.1; -0.1 0.5 1.1; -0.1 0.5 1.1];  % Example parameters

    %% Generate FIS
    for a = 1:n_a

        fis = sugfis;

        for i = 1:numel(inputs)
            fis = addInput(fis, [0 1], 'Name', inputs(i));
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

    ruleList = [
      % Low t_response, Low Priority -> Medium Attraction
      1 1 2 1 1;
      % Low t_response, Medium Priority -> High Attraction
      1 2 3 1 1;
      % Low t_response, High Priority -> High Attraction
      1 3 3 1 1;
      
      % Medium t_response, Low Priority -> Low Attraction
      2 1 1 1 1;
      % Medium t_response, Medium Priority -> Medium Attraction
      2 2 2 1 1;
      % Medium t_response, High Priority -> High Attraction
      2 3 3 1 1;
      
      % High t_response, Low Priority -> Low Attraction
      3 1 1 1 1;
      % High t_response, Medium Priority -> Medium Attraction
      3 2 2 1 1;
      % High t_response, High Priority -> Medium Attraction
      3 3 2 1 1;
    ];


        fis = addRule(fis, ruleList);

        %% Add to FIS array
        fisArray(a) = fis;
    end
end
