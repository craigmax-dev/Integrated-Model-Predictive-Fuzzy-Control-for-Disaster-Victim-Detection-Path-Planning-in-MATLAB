function mpc_model = i_arch_mpfc_input_exact(fisArray, agent_model)
  % Architecture setup
  architecture = 'mpfc'; % Options: mpc, mpfc, fis
  structure = 'centralised'; % Options: centralised, decentralised, clustered (TBC)
  prediction_mode = 'deterministic_exact'; % Prediction model setup
  optimization_target = 'input'; % or 'input'

  % Initial parameter and constraints setup
  [ini_params, solver, options_firstEval, options_subsequentEval, A, b, lb, ub, intCon] = initSupervisoryController(architecture, structure, fisArray, agent_model, optimization_target);

  % Structuring mpc_model
  mpc_model = struct('architecture', architecture, 'structure', structure, 'solver', solver, 'intCon', intCon, ...
                     'options_firstEval', options_firstEval, 'options_subsequentEval', options_subsequentEval, ...
                     'n_p', 1, 'ini_params', ini_params, 'A', A, 'b', b, 'Aeq', [], 'beq', [], ...
                     'lb', lb, 'ub', ub, 'nonlcon', [], 'nvars', numel(ini_params), 'prediction_model', prediction_mode, ...
                     'mpc_tuning_parameters', 'input', 'optimizationTimes', [], 'optimization_target', optimization_target);
end

