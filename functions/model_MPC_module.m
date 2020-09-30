% Function model_MPC_module
% Runs MPC module
% Passes function handle of MPC model to chosen solver and returns 
% fisArray with optimised parameters

function [fisArray, ini_params, fis_param_hist] = ...
  model_MPC_module(fisArray, ini_params, fis_param_hist, ...
  solver, h_MPC, n_a, ...
  nvars, A, b, Aeq, beq, lb, ub, nonlcon, ...
  fminsearchOptions, gaOptions, patOptions, parOptions)
  % For reproducibility
  rng default;
  % Optimisation
  if solver == "fminsearch"
    [mpc_params, fval] = fminsearch(h_MPC, ini_params, fminsearchOptions);
  elseif solver == "ga"
    [mpc_params,fval] = ga(h_MPC, nvars, A, b, Aeq, beq, lb, ub, nonlcon, gaOptions);   
  elseif solver == "patternsearch"
    [mpc_params,fval] = patternsearch(h_MPC, ini_params, A, b, Aeq, beq, lb, ub, nonlcon, patOptions);   
  elseif solver == "particleswarm"
    [mpc_params,fval] = particleswarm(h_MPC, nvars, lb, ub, parOptions);   
  end
  % Update FIS Parameters
  for a=1:n_a
    range       = 1 + (a - 1) * 4;
    fis_params  = mpc_params(range:range+3);
    fisArray(a).Outputs.MembershipFunctions.Parameters = fis_params;
  end
  % Update initial guess
  ini_params = mpc_params;
  % Record new parameters
  fis_param_hist = [fis_param_hist; mpc_params(1:n_a*4)];
end