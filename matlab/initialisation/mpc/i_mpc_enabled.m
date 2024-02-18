%% Function initialise_MPC
% Initialise MPC model

% V2
% CHANGELOG
% Refactor: mpc_model structure

% TODO
% Review lb/ub constraints (needs to be within range of MFs?)
% Review advanced constriants: constrain to cover entire input range?

function mpc_model = i_mpc_enabled(fisArray, n_a)

  flag_mpc = true;
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
  lb      = zeros(size(ini_params)); % Lower bound of 0 for all parameters
  ub      = repmat(1000, size(ini_params)); % Replace 'max_value' with your actual upper limit
  nonlcon = [];
  nvars = size(ini_params, 2);

 
  %% Membership function constraints

  % Number of MFs per input
  numMFs = 3;
  
  % Total number of parameters
  totalParams = numMFs * 2 * 3; % 3 parameters per MF, 2 inputs
  
  % Initialize A and b
  A = [];
  b = [];
  
  % Monotonicity Constraints
  for i = 1:totalParams/3
      idx = (i-1)*3 + 1;
      A_temp = zeros(2, totalParams);
      A_temp(1, idx:idx+1) = [1, -1]; % a - b <= 0
      A_temp(2, idx+1:idx+2) = [1, -1]; % b - c <= 0
      A = [A; A_temp];
      b = [b; 0; 0];
  end

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