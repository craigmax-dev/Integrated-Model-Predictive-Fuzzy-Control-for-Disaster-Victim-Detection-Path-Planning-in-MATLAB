% TO DO: rename initControllerParameters -> initSupervisoryController

function [ini_params, solver, options_firstEval, options_subsequentEval, A, b, lb, ub, intCon] = initControllerParameters(architecture, fisArray, agent_model, optimization_target)
    % Initialization depending on architecture
    switch architecture
        case 'mpc'
            ini_params = randomiseAgentTaskQueue(agent_model.n_a, agent_model.n_q, agent_model.n_x_s, agent_model.n_y_s);
            solver = 'ga';
            [A, b, lb, ub, intCon] = defineConstraintsForMPC(agent_model);
        case 'mpfc'
            if strcmp(optimization_target, 'input')
                ini_params = extractFISInputParameters(fisArray);
                [A, b, lb, ub, intCon] = defineConstraintsForMPFCInput(ini_params);
            else % 'output'
                ini_params = extractFISOutputParameters(fisArray);
                [A, b, lb, ub, intCon] = defineConstraintsForMPFCOutput(ini_params);
            end
            solver = 'patternsearch';
        case 'fis'
            % For 'fis', set placeholders as optimization is not required
            ini_params = [];
            solver = '';
            A = []; b = []; lb = []; ub = []; intCon = [];
        otherwise
            error('Invalid architecture value: %s', architecture);
    end
    
    % Set optimization options
    [options_firstEval, options_subsequentEval] = setOptimizationOptions(solver, architecture);
end


function [A, b, lb, ub, intCon] = defineConstraintsForMPC(agent_model)
    lb = ones(1, agent_model.n_a * agent_model.n_q * 2);
    ub = reshape([repmat(agent_model.n_x_s, 1, agent_model.n_a * agent_model.n_q); repmat(agent_model.n_y_s, 1, agent_model.n_a * agent_model.n_q)], 1, []);
    A = []; b = []; % No linear constraints
    intCon = 1:numel(lb); % All variables are integers
end

function [A, b, lb, ub, intCon] = defineConstraintsForMPFCInput(ini_params)
    % Constraints ensuring that input parameters allow for some rules to be always applicable
    lb = zeros(size(ini_params)); % Assuming all parameters have a lower bound of 0
    ub = repmat(100, size(ini_params)); % Adjust upper bound to ensure parameters stay within a range that allows rule firing
    A = []; b = []; % Additional constraints can be added here if needed
    intCon = []; % No integer constraints in MPFC for input parameters
end

function [A, b, lb, ub, intCon] = defineConstraintsForMPFCOutput(ini_params)
    % Constraints ensuring that output parameters allow for some rules to be always applicable
    % lb = repmat(-1, size(ini_params)); % Assume lower bound of -1
    % ub = repmat(1, size(ini_params)); % Assume upper bound of 1
    lb = -1 * ones(size(ini_params)); % Assume lower bound of -1
    ub = ones(size(ini_params)); % Assume upper bound of 1    
    A = []; b = []; % Additional constraints can be added here if needed
    intCon = []; % No integer constraints in MPFC for output parameters
end

function [options_firstEval, options_subsequentEval] = setOptimizationOptions(solver, architecture)
    switch architecture
        case 'mpc'
            options_firstEval = optimoptions(solver, 'UseParallel', false, 'PopulationSize', 100, 'MaxGenerations', 100, 'UseParallel', true);
            options_subsequentEval = options_firstEval; % Same options for simplicity
        case 'mpfc'
            options_firstEval = optimoptions(solver, 'MaxFunEvals', 100, 'MeshTolerance', 1e-3, 'StepTolerance', 1e-3);
            options_subsequentEval = options_firstEval; % Same options for simplicity
        otherwise
            options_firstEval = optimoptions('fmincon'); % Default options, won't be used
            options_subsequentEval = options_firstEval; % Same options for simplicity, won't be used
    end
end

function inputParams = extractFISInputParameters(fisArray)
    inputParams = [];
    for a = 1:numel(fisArray) % Iterate over each FIS
        fis = fisArray(a);
        for i = 1:numel(fis.Inputs) % Iterate over each input
            for mf = 1:numel(fis.Inputs(i).MembershipFunctions) % Iterate over each MF
                mfParams = fis.Inputs(i).MembershipFunctions(mf).Parameters;
                inputParams = [inputParams, mfParams];
            end
        end
    end
end


function outputParams = extractFISOutputParameters(fisArray)
    outputParams = [];
    for a = 1:numel(fisArray) % Iterate over each FIS
        fis = fisArray(a);
        for o = 1:numel(fis.Outputs) % Iterate over each output
            for mf = 1:numel(fis.Outputs(o).MembershipFunctions) % Iterate over each MF
                mfParams = fis.Outputs(o).MembershipFunctions(mf).Parameters;
                outputParams = [outputParams, mfParams];
            end
        end
    end
end
