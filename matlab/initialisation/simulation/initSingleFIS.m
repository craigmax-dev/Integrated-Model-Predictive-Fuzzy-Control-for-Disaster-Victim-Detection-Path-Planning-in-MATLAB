
% V5
function fis = initSingleFIS(inputs, inputRanges, mfTypes, mfNames, inputPreferences)
    fis = sugfis('Name', 'DecisionFIS');
    for i = 1:length(inputs)
        fis = addInput(fis, inputRanges(i,:), 'Name', inputs(i));
        for mfIndex = 1:length(mfNames)
            params = calculateMFParams(inputRanges(i,:), mfIndex, length(mfNames));
            fis = addMF(fis, inputs(i), mfTypes(i), params, 'Name', mfNames(mfIndex));
        end
    end

    % Configure outputs and rules based on input preferences
    fis = configureOutputAndRules(fis, inputPreferences);
end

function fis = configureOutputAndRules(fis, inputPreferences)
    outputName = "attraction";
    outputRange = [0 1];
    fis = addOutput(fis, outputRange, 'Name', outputName);

    numInputs = numel(fis.Inputs);  % Get the number of inputs    

    % Adjust output MF parameters based on input preferences
    coefficients = zeros(1, numInputs);
    for i = 1:numInputs
        if strcmp(inputPreferences{i}, 'minimize')
            coefficients(i) = -1;
        elseif strcmp(inputPreferences{i}, 'maximize')
            coefficients(i) = 1;
        else
            coefficients(i) = 0;
        end
    end

    % Add output membership functions with adjusted coefficients
    fis = addMF(fis, outputName, 'linear', [coefficients 0], 'Name', 'low');
    fis = addMF(fis, outputName, 'linear', [coefficients 0.5], 'Name', 'medium');
    fis = addMF(fis, outputName, 'linear', [coefficients 1], 'Name', 'high');


    % Generate and add rules based on the input preferences
    ruleList = generateRules(fis, inputPreferences);
    for i = 1:numel(ruleList)
        fis = addRule(fis, ruleList{i});
    end
end

% function ruleList = generateRules(fis, inputPreferences)
%     numInputs = numel(fis.Inputs);
%     ruleList = {};
%     outputName = "attraction";
% 
%     % For each output MF, generate a rule that applies based on input preferences
%     outputMFNames = {'low', 'medium', 'high'};
%     for outputIdx = 1:numel(outputMFNames)
%         conditions = cell(1, numInputs);
%         for inputIdx = 1:numInputs
%             if strcmp(inputPreferences{inputIdx}, 'minimize')
%                 conditions{inputIdx} = sprintf('(%s is %s)', fis.Inputs(inputIdx).Name, fis.Inputs(inputIdx).MembershipFunctions(1).Name);
%             elseif strcmp(inputPreferences{inputIdx}, 'maximize')
%                 conditions{inputIdx} = sprintf('(%s is %s)', fis.Inputs(inputIdx).Name, fis.Inputs(inputIdx).MembershipFunctions(end).Name);
%             else
%                 conditions{inputIdx} = sprintf('(%s is %s)', fis.Inputs(inputIdx).Name, fis.Inputs(inputIdx).MembershipFunctions(round(end/2)).Name);
%             end
%         end
%         ruleText = sprintf('IF %s THEN %s is %s (1.0)', strjoin(conditions, ' AND '), outputName, outputMFNames{outputIdx});
%         ruleList{end+1} = ruleText;
%     end
% end

function ruleList = generateRules(fis, inputPreferences)
    numInputs = numel(fis.Inputs);
    % Retrieve the number of membership functions for each input
    numMFsEachInput = arrayfun(@(input) numel(input.MembershipFunctions), fis.Inputs);

    % Generate all combinations of MF indices
    mfIndexCombinations = allComb(numMFsEachInput);

    ruleList = {};
    outputName = "attraction";
    outputMFNames = {'low', 'medium', 'high'};

    % Iterate over all combinations of input MFs
    for idx = 1:size(mfIndexCombinations, 1)
        mfIndices = mfIndexCombinations(idx, :);
        conditions = cell(1, numInputs);
        scores = zeros(1, numInputs);  % Scores to determine which output MF to use

        % Construct rule conditions based on input MF indices and preferences
        for inputIdx = 1:numInputs
            currentMFIndex = mfIndices(inputIdx);
            currentMF = fis.Inputs(inputIdx).MembershipFunctions(currentMFIndex).Name;

            conditions{inputIdx} = sprintf('(%s is %s)', fis.Inputs(inputIdx).Name, currentMF);

            % Determine influence score based on preferences
            switch inputPreferences{inputIdx}
                case 'minimize'
                    % Negative influence
                    scores(inputIdx) = -currentMFIndex;
                case 'maximize'
                    % Positive influence
                    scores(inputIdx) = currentMFIndex;
                otherwise
                    % Neutral influence
                    scores(inputIdx) = 0;
            end
        end

        % Calculate the aggregate score and decide the output MF
        totalScore = sum(scores);
        if totalScore < 0
            outputMF = 'low';
        elseif totalScore > 0
            outputMF = 'high';
        else
            outputMF = 'medium';
        end

        ruleText = sprintf('IF %s THEN %s is %s (1.0)', strjoin(conditions, ' AND '), outputName, outputMF);
        ruleList{end+1} = ruleText;
    end
end

function combinations = allComb(numMFsEachInput)
    % Generate all possible combinations of MF indices for each input
    grid = cell(1, numel(numMFsEachInput));
    for i = 1:numel(numMFsEachInput)
        grid{i} = 1:numMFsEachInput(i);
    end
    [grid{:}] = ndgrid(grid{:});
    combinations = cell2mat(cellfun(@(x) x(:), grid, 'UniformOutput', false));
    combinations = combinations(:, end:-1:1); % Correct order to match input sequence
end



function params = calculateMFParams(inputRange, mfIndex, totalMFs)
    step = (inputRange(2) - inputRange(1)) / (totalMFs - 1);
    params = [max(inputRange(1), inputRange(1) + step * (mfIndex - 2)), ...
              inputRange(1) + step * (mfIndex - 1), ...
              min(inputRange(2), inputRange(1) + step * mfIndex)];
end



% V4

% function fis = initSingleFIS(inputs, inputRanges, mfTypes, mfNames, inputPreferences)
%     fis = sugfis;
%     for i = 1:length(inputs)
%         fis = addInput(fis, inputRanges(i,:), 'Name', inputs(i));
%         for mfIndex = 1:length(mfNames)
%             params = calculateMFParams(inputRanges(i,:), mfIndex, length(mfNames));
%             fis = addMF(fis, inputs(i), mfTypes(i), params, 'Name', mfNames(mfIndex));
%         end
%     end
% 
%     % Configure outputs and rules based on input preferences
%     fis = configureOutputAndRules(fis, inputPreferences);
% end
% 
% function fis = configureOutputAndRules(fis, inputPreferences)
% 
%     outputName = "attraction";
%     outputRange = [0 1];
%     fis = addOutput(fis, outputRange, 'Name', outputName);
% 
%     numInputs = numel(fis.Inputs);  % Get the number of inputs    
% 
%     % Define parameters for linear output MFs based on number of inputs
%     lowParams = [zeros(1, numInputs) 0];  % Lowest value
%     mediumParams = [zeros(1, numInputs) 0.5];  % Middle value
%     highParams = [zeros(1, numInputs) 1];      % Highest value
% 
%     % Add output membership functions
%     fis = addMF(fis, outputName, 'linear', lowParams, 'Name', 'low');
%     fis = addMF(fis, outputName, 'linear', mediumParams, 'Name', 'medium');
%     fis = addMF(fis, outputName, 'linear', highParams, 'Name', 'high');
% 
%     % Generate and add rules based on the input preferences
%     ruleList = generateRules(fis, inputPreferences);
%     for i = 1:size(ruleList, 1)
%         fis = addRule(fis, ruleList{i});
%     end
% end
% 
% function ruleList = generateRules(fis, inputPreferences)
%     numInputs = numel(fis.Inputs);
%     numMFsEachInput = arrayfun(@(input) numel(input.MembershipFunctions), fis.Inputs);
% 
%     % Generate all combinations of MF indices
%     mfIndexCombinations = allComb(numMFsEachInput);
%     ruleList = [];
% 
%     for idx = 1:size(mfIndexCombinations, 1)
%         mfIndices = mfIndexCombinations(idx, :);
%         coeffs = zeros(1, numInputs);  % Initialize coefficients
% 
%         % Determine coefficients based on input preferences
%         for n_inputs = 1:numInputs
%             if strcmp(inputPreferences{n_inputs}, 'minimize')
%                 coeffs(n_inputs) = -1;  % Negative influence
%             elseif strcmp(inputPreferences{n_inputs}, 'maximize')
%                 coeffs(n_inputs) = 1;   % Positive influence
%             else
%                 coeffs(n_inputs) = 0;   % Neutral has no influence
%             end
%         end
% 
%         % Combine coefficients with the input MF indices to form the rule
%         conditionParts = arrayfun(@(idx, coeff) sprintf('input%d is %s (%.1f)', idx, fis.Inputs(idx).MembershipFunctions(mfIndices(idx)).Name, coeff), ...
%                                   1:numInputs, coeffs, 'UniformOutput', false);
%         ruleText = sprintf('IF %s THEN attraction is medium (1.0)', strjoin(conditionParts, ' AND '));
%         ruleList{end+1} = ruleText;
%     end
% end
% 
% 
% 
% function combinations = allComb(numMFsEachInput)
%     % Generate all possible combinations of MF indices for each input
%     grid = cell(1, numel(numMFsEachInput));
%     for i = 1:numel(numMFsEachInput)
%         grid{i} = 1:numMFsEachInput(i);
%     end
%     [grid{:}] = ndgrid(grid{:});
%     combinations = cell2mat(cellfun(@(x) x(:), grid, 'UniformOutput', false));
%     combinations = combinations(:, end:-1:1); % Correct order to match input sequence
% end
% 
% function params = calculateMFParams(inputRange, mfIndex, totalMFs)
%     step = (inputRange(2) - inputRange(1)) / (totalMFs - 1);
%     params = [max(inputRange(1), inputRange(1) + step * (mfIndex - 2)), ...
%               inputRange(1) + step * (mfIndex - 1), ...
%               min(inputRange(2), inputRange(1) + step * mfIndex)];
% end


% V3 

% CHANGELOG
% Implemented robust config of output MFs for provided inputs

% function fis = initSingleFIS(inputs, inputRanges, mfTypes, mfNames, inputPreferences)
%     fis = sugfis;
%     for i = 1:length(inputs)
%         fis = addInput(fis, inputRanges(i,:), 'Name', inputs(i));
%         for mfIndex = 1:length(mfNames)
%             params = calculateMFParams(inputRanges(i,:), mfIndex, length(mfNames));
%             fis = addMF(fis, inputs(i), mfTypes(i), params, 'Name', mfNames(mfIndex));
%         end
%     end
% 
%     % Configure outputs and rules based on input preferences
%     fis = configureOutputAndRules(fis, inputPreferences);
% end
% function fis = configureOutputAndRules(fis, inputPreferences)
%     outputName = "attraction";
%     outputRange = [0 1];
%     fis = addOutput(fis, outputRange, 'Name', outputName);
% 
%     numInputs = numel(fis.Inputs);  % Get the number of inputs
% 
%     % Define parameters for linear output MFs based on number of inputs
%     lowParams = [zeros(1, numInputs) 0];  % Lowest value
%     mediumParams = [zeros(1, numInputs) 0.5];  % Middle value
%     highParams = [zeros(1, numInputs) 1];      % Highest value
% 
%     % Add output membership functions
%     fis = addMF(fis, outputName, 'linear', lowParams, 'Name', 'low');
%     fis = addMF(fis, outputName, 'linear', mediumParams, 'Name', 'medium');
%     fis = addMF(fis, outputName, 'linear', highParams, 'Name', 'high');
% 
%     % Generate rules based on input preferences
%     ruleTexts = generateRuleTexts(fis, inputPreferences, outputName);
%     for i = 1:length(ruleTexts)
%         fis = addRule(fis, ruleTexts{i});
%     end
% end
% 
% function ruleTexts = generateRuleTexts(fis, inputPreferences, outputName)
%     numInputs = numel(fis.Inputs);
%     ruleTexts = cell(1, numInputs);
%     for i = 1:numInputs
%         inputName = fis.Inputs(i).Name;
%         % Select output membership function based on input preference
%         switch inputPreferences{i}
%             case 'minimize'
%                 consequence = 'low';
%             case 'maximize'
%                 consequence = 'high';
%             case 'neutral'
%                 consequence = 'medium';
%         end
%         % Construct rule text
%         ruleTexts{i} = sprintf('IF %s is %s THEN %s is %s', inputName, 'medium', outputName, consequence);
%     end
% end
% 
% function params = calculateMFParams(inputRange, mfIndex, totalMFs)
%     step = (inputRange(2) - inputRange(1)) / (totalMFs - 1);
%     params = [max(inputRange(1), inputRange(1) + step * (mfIndex - 2)), ...
%               inputRange(1) + step * (mfIndex - 1), ...
%               min(inputRange(2), inputRange(1) + step * mfIndex)];
% end



% % V2
% function fis = initSingleFIS(inputs, inputRanges, mfTypes, mfNames, inputPreferences)
%     fis = sugfis;
%     for i = 1:length(inputs)
%         fis = addInput(fis, inputRanges(i,:), 'Name', inputs(i));
%         for mfIndex = 1:length(mfNames)
%             params = calculateMFParams(inputRanges(i,:), mfIndex, length(mfNames));
%             fis = addMF(fis, inputs(i), mfTypes(i), params, 'Name', mfNames(mfIndex));
%         end
%     end
% 
%     % Correctly pass inputPreferences to configureOutputAndRules
%     fis = configureOutputAndRules(fis, inputPreferences);
% end
% 
% function fis = configureOutputAndRules(fis, inputPreferences)
%     outputName = "attraction";
%     outputRange = [0 1];
%     fis = addOutput(fis, outputRange, 'Name', outputName);
% 
%     % Correctly determine the number of inputs to the FIS
%     numInputs = numel(fis.Inputs); % This should correctly count the number of inputs
% 
%     % Example adjusted parameters for linear MFs (assuming 2 inputs for simplicity)
%     outputMFs = {'linear', [1 0.5 0], 'low'; 
%                  'linear', [0.5 1 0], 'medium'; 
%                  'linear', [1 1 0.5], 'high'};
% 
%     for i = 1:size(outputMFs, 1)
%         % Assuming linear MF requires 'numInputs + 1' parameters
%         mfParams = zeros(1, numInputs + 1); % Adjusted to use correct 'numInputs'
%         fis = addMF(fis, outputName, outputMFs{i,1}, mfParams, 'Name', outputMFs{i,3});
%     end
% 
%     fis = addRule(fis, generateRules(fis, inputPreferences));
% end
% 
% function params = calculateMFParams(inputRange, mfIndex, totalMFs)
%     % Calculates the parameters for a membership function based on the input range and the index of the membership function.
%     %
%     % Parameters:
%     %   inputRange: A 2-element vector specifying the start and end of the range for the input.
%     %   mfIndex: The index of the membership function for which parameters are being calculated.
%     %   totalMFs: The total number of membership functions defined for the input.
%     %
%     % Returns:
%     %   params: A vector of parameters for the specified membership function.
% 
%     step = (inputRange(2) - inputRange(1)) / (totalMFs - 1);
%     params = [max(inputRange(1), inputRange(1) + step * (mfIndex - 2)), ...
%               inputRange(1) + step * (mfIndex - 1), ...
%               min(inputRange(2), inputRange(1) + step * mfIndex)];
% end
% 
% function ruleList = generateRules(fis, inputPreferences)
%     numInputs = numel(fis.Inputs);
%     numMFsEachInput = arrayfun(@(input) numel(input.MembershipFunctions), fis.Inputs);
%     maxScore = max(numMFsEachInput);
% 
%     % Generate all combinations of MF indices
%     mfIndexCombinations = allComb(numMFsEachInput);
% 
%     ruleList = [];
%     for idx = 1:size(mfIndexCombinations, 1)
%         mfIndices = mfIndexCombinations(idx, :);
%         scores = zeros(1, numInputs);
% 
%         for n_inputs = 1:numInputs
%             currentIndex = mfIndices(n_inputs);
%             if strcmp(inputPreferences(n_inputs), 'minimize')
%                 scores(n_inputs) = maxScore - (currentIndex - 1);
%             elseif strcmp(inputPreferences(n_inputs), 'maximize')
%                 scores(n_inputs) = currentIndex;
%             elseif strcmp(inputPreferences(n_inputs), 'neutral')
%               % Assign a middle score for neutral preference
%               scores(n_inputs) = round(maxScore / 2);
%             end
%         end
% 
%         avgScore = round(mean(scores));
%         outputMFIndex = avgScore;
%         rule = [mfIndices, outputMFIndex, 1, 1];
%         ruleList = [ruleList; rule];
%     end
% end
% 
% function combinations = allComb(numMFsEachInput)
%     % Generate all possible combinations of MF indices for each input
%     grid = cell(1, numel(numMFsEachInput));
%     for i = 1:numel(numMFsEachInput)
%         grid{i} = 1:numMFsEachInput(i);
%     end
%     [grid{:}] = ndgrid(grid{:});
%     combinations = cell2mat(cellfun(@(x) x(:), grid, 'UniformOutput', false));
%     combinations = combinations(:, end:-1:1); % Correct order to match input sequence
% end

