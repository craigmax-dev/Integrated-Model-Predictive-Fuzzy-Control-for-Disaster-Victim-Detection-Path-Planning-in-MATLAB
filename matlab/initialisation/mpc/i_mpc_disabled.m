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
  % "deterministic_exact" - In this mode, the MPC predicts the fire spread exactly as it will occur in the simulation. It assumes perfect knowledge of how the fire will evolve over the prediction horizon, making it the most accurate but potentially the least flexible mode. This mode is ideal for scenarios where the fire spread model is highly reliable and environmental conditions are well-understood.
  % "deterministic_prediction" - This mode also uses a deterministic approach to predict fire spread but allows for different permutations of fire evolution compared to what actually occurs in the simulation. It's useful for exploring various "what-if" scenarios within the planning horizon, providing insights into different possible outcomes based on slight variations in initial conditions or fire behavior.
  % "probabilistic_threshold" - Unlike the deterministic modes, this mode introduces a probabilistic element to the prediction. It sets a threshold probability for fire spread; if the calculated probability of fire spreading to a cell is above this threshold, the model predicts that the fire will propagate to that cell. This mode is beneficial for incorporating uncertainty into the prediction, allowing for more robust planning under uncertain environmental conditions.
  prediction_model = "deterministic_exact"; 

  % Structure
  mpc_model = struct('architecture', architecture, 'flag_mpc', flag_mpc, 'solver', solver, 'options_firstEval', options_firstEval, 'options_subsequentEval', options_subsequentEval, ...
  'n_p', n_p, 'fis_params', fis_params, 'ini_params', ini_params, 'A', A, 'b', b, 'Aeq', Aeq, 'beq', beq, 'lb', lb, 'ub', ub, 'nonlcon', nonlcon, ...
  'nvars', nvars, 'prediction_model', prediction_model);

end