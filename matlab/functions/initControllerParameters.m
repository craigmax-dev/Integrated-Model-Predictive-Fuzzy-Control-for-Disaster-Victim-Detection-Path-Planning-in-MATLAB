% V2.2
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
            else % 'output'
                ini_params = extractFISOutputParameters(fisArray);
            end
            solver = 'patternsearch';
            [A, b, lb, ub, intCon] = defineConstraintsForMPFC(ini_params, optimization_target);
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

function [A, b, lb, ub, intCon] = defineConstraintsForMPFC(ini_params, optimizeWhat)
    lb = zeros(size(ini_params)); % Assuming all parameters have a lower bound of 0
    ub = repmat(1000, size(ini_params)); % Assuming a generic upper bound for all parameters
    A = []; b = []; % No linear constraints
    intCon = []; % No integer constraints in MPFC
end

function [options_firstEval, options_subsequentEval] = setOptimizationOptions(solver, architecture)
    switch architecture
        case 'mpc'
            options_firstEval = optimoptions(solver, 'Display', 'iter', 'PopulationSize', 500, 'MaxGenerations', 100, 'UseParallel', true);
            options_subsequentEval = options_firstEval; % Same options for simplicity
        case 'mpfc'
            options_firstEval = optimoptions(solver, 'Display', 'iter', 'MaxFunEvals', 500, 'MeshTolerance', 1e-3, 'StepTolerance', 1e-3);
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


% V2.1
% function [ini_params, solver, options_firstEval, options_subsequentEval, A, b, lb, ub, intCon] = initControllerParameters(architecture, fisArray, agent_model)
%     if strcmp(architecture, 'mpc')
%         % 'mpc' specific setup
%         ini_params = randomiseAgentTaskQueue(agent_model.n_a, agent_model.n_q, agent_model.n_x_s, agent_model.n_y_s);
%         solver = 'ga';
%         options_firstEval = optimoptions(solver, 'Display', 'iter', 'PopulationSize', 500, 'MaxGenerations', 100, 'UseParallel', true);
%         options_subsequentEval = options_firstEval; % Same options for simplicity
%         [A, b, lb, ub, intCon] = defineConstraints(architecture, fisArray, agent_model, ini_params);
%     elseif strcmp(architecture, 'mpfc')
%         % 'mpfc' specific setup
%         ini_params = extractFISParameters(fisArray);
%         solver = 'patternsearch';
%         options_firstEval = optimoptions(solver, 'Display', 'iter', 'MaxFunEvals', 500, 'MeshTolerance', 1e-3, 'StepTolerance', 1e-3);
%         options_subsequentEval = options_firstEval; % Same options for simplicity
%         [A, b, lb, ub, intCon] = defineConstraints(architecture, fisArray, agent_model, ini_params);
%     elseif strcmp(architecture, 'fis')
%         % 'fis' specific setup - no optimization required
%         ini_params = []; % No initial parameters required for optimization
%         solver = ''; % No solver needed for 'fis'
%         options_firstEval = optimoptions('fmincon'); % Default options, won't be used
%         options_subsequentEval = options_firstEval; % Same options for simplicity, won't be used
%         A = []; % No constraints for 'fis'
%         b = [];
%         lb = [];
%         ub = [];
%         intCon = []; % No integer constraints for 'fis'
%     else
%         error("Invalid architecture value: %s", architecture);
%     end
% end
% 
% function fis_params = extractFISParameters(fisArray)
%     fis_params = [];
%     for a = 1:numel(fisArray)
%         for o = 1:numel(fisArray(a).Outputs)
%             for mf = 1:numel(fisArray(a).Outputs(o).MembershipFunctions)
%                 params = fisArray(a).Outputs(o).MembershipFunctions(mf).Parameters;
%                 fis_params = [fis_params, params];
%             end
%         end
%     end
% end
% 
% function [A, b, lb, ub, intCon] = defineConstraints(architecture, fisArray, agent_model, ini_params)
%     if strcmp(architecture, 'mpc')
%         lb = ones(1, agent_model.n_a * agent_model.n_q * 2); % Lower bounds
%         ub = reshape([repmat(agent_model.n_x_s, 1, agent_model.n_a * agent_model.n_q); repmat(agent_model.n_y_s, 1, agent_model.n_a * agent_model.n_q)], 1, []); % Upper bounds
%         A = []; % Specific A matrix setup for mpc
%         b = []; % Specific b vector setup for mpc
%         intCon = 1:numel(ini_params); % All variables are integers
%     elseif strcmp(architecture, 'mpfc')
%         % Example constraint setup for MPFC, adjust as necessary
%         lb = zeros(size(ini_params)); % Lower bound for FIS parameters
%         ub = repmat(1000, size(ini_params)); % Upper bound for FIS parameters
%         A = []; % Adjust based on MPFC constraints
%         b = []; % Adjust based on MPFC constraints
%         intCon = []; % Define based on MPFC constraints
%     else
%         lb = [];
%         ub = [];
%         A = [];
%         b = [];
%         intCon = [];
%     end
% end