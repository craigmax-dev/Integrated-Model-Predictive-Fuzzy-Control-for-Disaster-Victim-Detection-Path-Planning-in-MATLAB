%% Function initialise_MPC
% Initialise MPC model

% V2
% CHANGELOG
% Refactor: mpc_model structure

function mpc_model = i_mpc_disabled(fisArray, n_a)

  flag_mpc = false;
  n_p = 1;                  % Prediction horizon
  solver = "patternsearch"; % Solver options: fmincon, ga, particleswarm, patternsearch
  
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

  % Solver options
  optTermCond       = 'MaxTime';
  optTermCond_value = 60;
  options        = optimoptions('patternsearch','Display','iter', optTermCond, optTermCond_value);

  % Structure
  mpc_model = struct('flag_mpc', flag_mpc, 'solver', solver, 'options', options, ...
  'n_p', n_p, 'fis_params', fis_params, 'ini_params', ini_params, 'A', A, 'b', b, 'Aeq', Aeq, 'beq', beq, 'lb', lb, 'ub', ub, 'nonlcon', nonlcon, ...
  'nvars', nvars);

end