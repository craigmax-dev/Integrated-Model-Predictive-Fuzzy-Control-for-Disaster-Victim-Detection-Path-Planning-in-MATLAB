%% Function initialise_MPC
% Initialise MPC model

% V2
% CHANGELOG
% Refactor: mpc_model structure
% Feature: mpc and mpfc architectures

function mpc_model = i_arch_fis(fisArray, agent_model)
    % Architecture setup
    architecture = 'fis'; % Options: mpc, mpfc, fis
    prediction_model = 'deterministic_exact'; % Prediction model setup

    % Initial parameter and constraints setup
    [ini_params, solver, options_firstEval, options_subsequentEval, A, b, lb, ub, intCon] = initControllerParameters(architecture, fisArray, agent_model);

    % Structuring mpc_model
    mpc_model = struct('architecture', architecture, 'solver', solver, 'intCon', intCon, ...
                       'options_firstEval', options_firstEval, 'options_subsequentEval', options_subsequentEval, ...
                       'n_p', 1, 'ini_params', ini_params, 'A', A, 'b', b, 'Aeq', [], 'beq', [], ...
                       'lb', lb, 'ub', ub, 'nonlcon', [], 'nvars', numel(ini_params), 'prediction_model', prediction_model, ...
                       'mpc_tuning_parameters', 'input', 'optimizationTimes', []);
end
