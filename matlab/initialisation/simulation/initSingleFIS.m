
function fis = initSingleFIS(inputs, inputRanges, mfTypes, mfNames, inputPreferences)

    % Initializes a single Fuzzy Inference System (FIS) with specified inputs and membership functions.
    %
    % Parameters:
    %   inputs: A string array specifying the names of the inputs.
    %   inputRanges: A matrix specifying the range of each input.
    %   mfTypes: A string array specifying the types of membership functions for each input.
    %   mfNames: A string array specifying the names of the membership functions.
    %
    % Returns:
    %   fis: The initialized FIS object.

    fis = sugfis;
    for i = 1:length(inputs)
        fis = addInput(fis, inputRanges(i,:), 'Name', inputs(i));
        for mfIndex = 1:length(mfNames)
            params = calculateMFParams(inputRanges(i,:), mfIndex, length(mfNames));
            fis = addMF(fis, inputs(i), mfTypes(i), params, 'Name', mfNames(mfIndex));
        end
    end
    
    % Configure the outputs and rules within the same function for simplicity
    fis = configureOutputAndRules(fis, inputPreferences);
end

function fis = configureOutputAndRules(fis, inputPreferences)
    % Configures the output variable and adds rules to the FIS.
    %
    % Parameters:
    %   fis: The FIS object to which the output variable and rules will be added.
    %
    % Returns:
    %   fis: The updated FIS object with the output variable and rules configured.

    outputName = "attraction";
    outputRange = [0 1];
    fis = addOutput(fis, outputRange, 'Name', outputName);
    
    outputMFs = {'linear', [0 0 0], 'low'; 'linear', [0 0 0.5], 'medium'; 'linear', [0 0 1], 'high'};
    for i = 1:size(outputMFs, 1)
        fis = addMF(fis, outputName, outputMFs{i,1}, outputMFs{i,2}, 'Name', outputMFs{i,3});
    end
    
    fis = addRule(fis, generateRules(fis, inputPreferences));
end

function params = calculateMFParams(inputRange, mfIndex, totalMFs)
    % Calculates the parameters for a membership function based on the input range and the index of the membership function.
    %
    % Parameters:
    %   inputRange: A 2-element vector specifying the start and end of the range for the input.
    %   mfIndex: The index of the membership function for which parameters are being calculated.
    %   totalMFs: The total number of membership functions defined for the input.
    %
    % Returns:
    %   params: A vector of parameters for the specified membership function.

    step = (inputRange(2) - inputRange(1)) / (totalMFs - 1);
    params = [max(inputRange(1), inputRange(1) + step * (mfIndex - 2)), ...
              inputRange(1) + step * (mfIndex - 1), ...
              min(inputRange(2), inputRange(1) + step * mfIndex)];
end

function ruleList = generateRules(fis, inputPreferences)
    numInputs = numel(fis.Inputs);
    numMFsEachInput = arrayfun(@(input) numel(input.MembershipFunctions), fis.Inputs);
    maxScore = max(numMFsEachInput);

    % Generate all combinations of MF indices
    mfIndexCombinations = allComb(numMFsEachInput);
    
    ruleList = [];
    for idx = 1:size(mfIndexCombinations, 1)
        mfIndices = mfIndexCombinations(idx, :);
        scores = zeros(1, numInputs);

        for n_inputs = 1:numInputs
            currentIndex = mfIndices(n_inputs);
            if strcmp(inputPreferences(n_inputs), 'minimize')
                scores(n_inputs) = maxScore - (currentIndex - 1);
            else
                scores(n_inputs) = currentIndex;
            end
        end

        avgScore = round(mean(scores));
        outputMFIndex = avgScore;
        rule = [mfIndices, outputMFIndex, 1, 1];
        ruleList = [ruleList; rule];
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

function outputMFIndex = determineOutputIndex(input1MFIndex, input2MFIndex, numMFsEachInput, inputPreferences)
    % A custom logic to determine the output membership function index based on input preferences
    % For simplicity, this example maximizes the output when t_response is minimized and priority is maximized
    
    % Assuming the first input is 't_response' and the second is 'priority'
    % The logic can be adjusted based on specific requirements and preferences
    if input1MFIndex <= round(numMFsEachInput(1)/2) && input2MFIndex >= round(numMFsEachInput(2)/2)
        outputMFIndex = 3; % High attraction for low t_response and high priority
    elseif input1MFIndex >= round(numMFsEachInput(1)/2) && input2MFIndex <= round(numMFsEachInput(2)/2)
        outputMFIndex = 1; % Low attraction for high t_response and low priority
    else
        outputMFIndex = 2; % Medium attraction for other combinations
    end
end
