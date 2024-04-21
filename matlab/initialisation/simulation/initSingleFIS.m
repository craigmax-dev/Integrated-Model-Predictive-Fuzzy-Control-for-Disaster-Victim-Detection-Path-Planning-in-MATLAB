
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

% V3
function ruleList = generateRules(fis, inputPreferences)
    numInputs = numel(fis.Inputs);
    numOutputs = numel(fis.Outputs(1).MembershipFunctions); % Assuming all outputs have the same number of MFs
    
    % Retrieve the number of membership functions for each input
    numMFsEachInput = arrayfun(@(input) numel(input.MembershipFunctions), fis.Inputs);
    
    % Generate all combinations of MF indices
    mfIndexCombinations = allComb(numMFsEachInput);
    
    ruleList = {};
    outputName = "attraction";

    % Iterate over all combinations of input MFs
    for idx = 1:size(mfIndexCombinations, 1)
        mfIndices = mfIndexCombinations(idx, :);
        conditions = cell(1, numInputs);
        outputIndices = zeros(1, numInputs);  % Indices for output MFs based on preferences

        % Construct rule conditions and calculate output MF indices
        for inputIdx = 1:numInputs
            currentMFIndex = mfIndices(inputIdx);
            currentMF = fis.Inputs(inputIdx).MembershipFunctions(currentMFIndex).Name;
            conditions{inputIdx} = sprintf('(%s is %s)', fis.Inputs(inputIdx).Name, currentMF);

            % Map indices based on preferences
            if strcmp(inputPreferences{inputIdx}, 'minimize')
                % Inverse mapping for minimize
                outputIndices(inputIdx) = numMFsEachInput(inputIdx) - currentMFIndex + 1;
            elseif strcmp(inputPreferences{inputIdx}, 'maximize')
                % Direct mapping for maximize
                outputIndices(inputIdx) = currentMFIndex;
            else
                % Neutral should ideally not influence the decision or consider the middle MF
                outputIndices(inputIdx) = round(numMFsEachInput(inputIdx)/2);
            end
        end

        % Calculate the average index and round to nearest integer
        avgIndex = round(mean(outputIndices));
        % Ensure the index is within the bounds of output MFs
        avgIndex = max(1, min(avgIndex, numOutputs));

        % Get the corresponding output MF name
        outputMF = fis.Outputs(1).MembershipFunctions(avgIndex).Name;

        % Construct the rule text
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



% % V2
% function ruleList = generateRules(fis, inputPreferences)
%     numInputs = numel(fis.Inputs);
%     % Retrieve the number of membership functions for each input
%     numMFsEachInput = arrayfun(@(input) numel(input.MembershipFunctions), fis.Inputs);
% 
%     % Generate all combinations of MF indices
%     mfIndexCombinations = allComb(numMFsEachInput);
% 
%     ruleList = {};
%     outputName = "attraction";
%     outputMFNames = {'low', 'medium', 'high'};
% 
%     % Iterate over all combinations of input MFs
%     for idx = 1:size(mfIndexCombinations, 1)
%         mfIndices = mfIndexCombinations(idx, :);
%         conditions = cell(1, numInputs);
%         scores = zeros(1, numInputs);  % Scores to determine which output MF to use
% 
%         % Construct rule conditions based on input MF indices and preferences
%         for inputIdx = 1:numInputs
%             currentMFIndex = mfIndices(inputIdx);
%             currentMF = fis.Inputs(inputIdx).MembershipFunctions(currentMFIndex).Name;
% 
%             conditions{inputIdx} = sprintf('(%s is %s)', fis.Inputs(inputIdx).Name, currentMF);
% 
%             % Determine influence score based on preferences
%             if strcmp(inputPreferences{inputIdx}, 'minimize')
%                 % Negative influence - invert index influence
%                 scores(inputIdx) = -(numMFsEachInput(inputIdx) - currentMFIndex + 1);
%             elseif strcmp(inputPreferences{inputIdx}, 'maximize')
%                 % Positive influence
%                 scores(inputIdx) = currentMFIndex;
%             else
%                 % Neutral influence - no impact on scores
%                 scores(inputIdx) = 0;
%             end
%         end
% 
%         % Calculate the aggregate score and decide the output MF
%         totalScore = sum(scores);
%         if totalScore < 0
%             outputMF = 'low';
%         elseif totalScore > 0
%             outputMF = 'high';
%         else
%             outputMF = 'medium';
%         end
% 
%         ruleText = sprintf('IF %s THEN %s is %s (1.0)', strjoin(conditions, ' AND '), outputName, outputMF);
%         ruleList{end+1} = ruleText;
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


% % V1
% function ruleList = generateRules(fis, inputPreferences)
%     numInputs = numel(fis.Inputs);
%     % Retrieve the number of membership functions for each input
%     numMFsEachInput = arrayfun(@(input) numel(input.MembershipFunctions), fis.Inputs);
% 
%     % Generate all combinations of MF indices
%     mfIndexCombinations = allComb(numMFsEachInput);
% 
%     ruleList = {};
%     outputName = "attraction";
%     outputMFNames = {'low', 'medium', 'high'};
% 
%     % Iterate over all combinations of input MFs
%     for idx = 1:size(mfIndexCombinations, 1)
%         mfIndices = mfIndexCombinations(idx, :);
%         conditions = cell(1, numInputs);
%         scores = zeros(1, numInputs);  % Scores to determine which output MF to use
% 
%         % Construct rule conditions based on input MF indices and preferences
%         for inputIdx = 1:numInputs
%             currentMFIndex = mfIndices(inputIdx);
%             currentMF = fis.Inputs(inputIdx).MembershipFunctions(currentMFIndex).Name;
% 
%             conditions{inputIdx} = sprintf('(%s is %s)', fis.Inputs(inputIdx).Name, currentMF);
% 
%             % Determine influence score based on preferences
%             switch inputPreferences{inputIdx}
%                 case 'minimize'
%                     % Negative influence
%                     scores(inputIdx) = -currentMFIndex;
%                 case 'maximize'
%                     % Positive influence
%                     scores(inputIdx) = currentMFIndex;
%                 otherwise
%                     % Neutral influence
%                     scores(inputIdx) = 0;
%             end
%         end
% 
%         % Calculate the aggregate score and decide the output MF
%         totalScore = sum(scores);
%         if totalScore < 0
%             outputMF = 'low';
%         elseif totalScore > 0
%             outputMF = 'high';
%         else
%             outputMF = 'medium';
%         end
% 
%         ruleText = sprintf('IF %s THEN %s is %s (1.0)', strjoin(conditions, ' AND '), outputName, outputMF);
%         ruleList{end+1} = ruleText;
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

function params = calculateMFParams(inputRange, mfIndex, totalMFs)
    step = (inputRange(2) - inputRange(1)) / (totalMFs - 1);
    params = [max(inputRange(1), inputRange(1) + step * (mfIndex - 2)), ...
              inputRange(1) + step * (mfIndex - 1), ...
              min(inputRange(2), inputRange(1) + step * mfIndex)];
end


