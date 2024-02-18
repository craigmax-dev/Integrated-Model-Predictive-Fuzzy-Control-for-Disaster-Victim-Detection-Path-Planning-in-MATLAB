%% Function initialise_MPC
% Initialise MPC model

% V2
% CHANGELOG
% Refactor: mpc_model structure

% TODO
% Review constraints
% Implement constraints on low-med-high
% Review lb/ub constraints (needs to be within range of MFs?)
% Review advanced constriants: constrain to cover entire input range?

function mpc_model = i_mpc_enabled_1000(fisArray, n_a)

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
  optTermCond_value = 1000;
  options_firstEval = optimoptions('patternsearch','Display','iter', optTermCond, optTermCond_value);

  % Solver options for subsequent eval
  optTermCond       = 'MaxFunEvals';
  optTermCond_value = 1000;
  options_subsequentEval = optimoptions('patternsearch','Display','iter', optTermCond, optTermCond_value);

  % Prediction mode
  % "deterministic", "probabilistic", or "downwind_approximation"
  prediction_model = "deterministic"; 

  % Structure
  mpc_model = struct('architecture', architecture, 'flag_mpc', flag_mpc, 'solver', solver, 'options_firstEval', options_firstEval, 'options_subsequentEval', options_subsequentEval, ...
  'n_p', n_p, 'fis_params', fis_params, 'ini_params', ini_params, 'A', A, 'b', b, 'Aeq', Aeq, 'beq', beq, 'lb', lb, 'ub', ub, 'nonlcon', nonlcon, ...
  'nvars', nvars, 'prediction_model', prediction_model);

end