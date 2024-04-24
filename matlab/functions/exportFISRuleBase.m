function ruleBaseMatrix = exportFISRuleBase(fis)
    % Extract rules from the FIS
    rules = fis.Rules;
    numRules = numel(rules);
    numInputs = numel(fis.Inputs);
    numOutputs = numel(fis.Outputs);
    
    % Initialize the rule base matrix as a string array
    ruleBaseMatrix = strings(numRules, numInputs + numOutputs);
    
    % Process each rule to fill the matrix
    for ruleIndex = 1:numRules
        rule = rules(ruleIndex);
        
        % Fill input parts of the rule
        for inIndex = 1:numInputs
            mfIndex = rule.Antecedent(inIndex);
            if mfIndex > 0 % Check if there is a valid MF index
                ruleBaseMatrix(ruleIndex, inIndex) = fis.Inputs(inIndex).MembershipFunctions(mfIndex).Name;
            else
                ruleBaseMatrix(ruleIndex, inIndex) = "none"; % Indicate no MF used
            end
        end
        
        % Fill output parts of the rule
        for outIndex = 1:numOutputs
            mfIndex = rule.Consequent(outIndex);
            if mfIndex > 0 % Check if there is a valid MF index
                ruleBaseMatrix(ruleIndex, numInputs + outIndex) = fis.Outputs(outIndex).MembershipFunctions(mfIndex).Name;
            else
                ruleBaseMatrix(ruleIndex, numInputs + outIndex) = "none"; % Indicate no MF used
            end
        end
    end
end
