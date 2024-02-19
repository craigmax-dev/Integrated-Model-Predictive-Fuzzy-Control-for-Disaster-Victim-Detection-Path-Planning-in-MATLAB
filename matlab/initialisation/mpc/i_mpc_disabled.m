%% Function initialise_MPC
% Initialise MPC model

% V2
% CHANGELOG
% Refactor: mpc_model structure

function mpc_model = i_mpc_disabled(fisArray, n_a)

  flag_mpc = false;
  n_p = 1;                  % Prediction horizon
  solver = "patternsearch"; % Solver options: fmincon, ga, particleswarm, patternsearch
  architecture = "centralised"; % Architecture options: centralised, distributed, clustered
  
  % Initialise optimisation parameters
  fis_params = [];
  for a = 1:n_a
    fis_params = [fis_params, fisArray(a).Outputs.MembershipFunctions.Parameters];
  end
  ini_params = [];
  for i = 1:n_p
    ini_params = [ini_params, fis_params];
  end
  
  % Constraints
  A       = [];
  b       = [];
  Aeq     = [];
  beq     = [];
  lb      = [];
  ub      = [];
  nonlcon = [];
  nvars = size(ini_params, 2);

  % Solver options for first eval
  optTermCond       = 'MaxFunEvals';
  optTermCond_value = 200;
  options_firstEval = optimoptions('patternsearch','Display','iter', optTermCond, optTermCond_value);

  % Solver options for subsequent eval
  optTermCond       = 'MaxFunEvals';
  optTermCond_value = 200;
  options_subsequentEval = optimoptions('patternsearch','Display','iter', optTermCond, optTermCond_value);

  % Prediction mode
  % "deterministic_exact" - In the “Deterministic Exact ” mode, the exact environment parameters that occur in the simulation are passed to the MPC.
  % "deterministic_prediction" - In the “Deterministic Prediction” mode, the environment parameters at the current timestep are passed to the MPC, which then uses them to predict the future evolution of the environment parameters over the prediction horizon.
  % "deterministic_threshold" - TBC - In the “Deterministic Threshold” mode, a predefined threshold and the environment parameters at the current timestep are passed to the MPC. The MPC then performs a prediction using the threshold to determine whether any probabilistic parameters are propagated.
  % "probabilistic" - TBC - In this proposed mode, the MPC predicts the probability of the propagation of all environment variables at each timestep. The difficulty is that many functions rely on a deterministic propagation of the probabilistic parameters.
  prediction_model = "deterministic_exact"; 

  % Structure
  mpc_model = struct('architecture', architecture, 'flag_mpc', flag_mpc, 'solver', solver, 'options_firstEval', options_firstEval, 'options_subsequentEval', options_subsequentEval, ...
  'n_p', n_p, 'fis_params', fis_params, 'ini_params', ini_params, 'A', A, 'b', b, 'Aeq', Aeq, 'beq', beq, 'lb', lb, 'ub', ub, 'nonlcon', nonlcon, ...
  'nvars', nvars, 'prediction_model', prediction_model);

end