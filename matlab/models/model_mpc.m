% Function model_MPC_module
% Runs MPC module
% Passes function handle of MPC model to chosen solver and returns 
% fisArray with optimised parameters

% Updates initial optimisation parameter guess with previous results

% TODO
% Rename: model_mpc


function [fisArray, ini_params, fis_param_hist] = ...
  model_mpc(fisArray, ini_params, fis_param_hist, ...
  solver, options, n_a, n_MF_out, ...
  nvars, A, b, Aeq, beq, lb, ub, nonlcon, ...
    test_fis_sensitivity, ...
    m_f, m_bo, m_bt, m_s, m_scan, m_t_scan, m_victim_scan, ...
    dk_a, dk_c, dk_e, dk_mpc, dt_s, k, seed, ...
    n_p, n_x_s, n_y_s, n_x_f, n_y_f, n_q, ...
    a_loc, a_target, a_task, a_t_trav, a_t_scan, ...
    l_x_s, l_y_s, c_f_s, ...
    c_fs_1, c_fs_2, v_as, v_w, ang_w, ...
    r_bo, r_fo, fis_data, config)

  % Function handle
  h_MPC = @(params)func_MPC_model(params, ...
    fisArray, test_fis_sensitivity, ...
    m_f, m_bo, m_bt, m_s, m_scan, m_t_scan, m_victim_scan, ...
    dk_a, dk_c, dk_e, dk_mpc, dt_s, k, seed, ...
    n_a, n_MF_out, n_p, n_x_s, n_y_s, n_x_f, n_y_f, n_q, ...
    a_loc, a_target, a_task, a_t_trav, a_t_scan, ...
    l_x_s, l_y_s, c_f_s, ...
    c_fs_1, c_fs_2, v_as, v_w, ang_w, ...
    r_bo, r_fo, fis_data, config);

  %   % For reproducibility
  %   rng(k);
  
  % Optimisation
  if solver == "fminsearch"
    [mpc_params, ~] = fminsearch(h_MPC, ini_params, options);
  elseif solver == "ga"
    [mpc_params,~] = ga(h_MPC, nvars, A, b, Aeq, beq, lb, ub, nonlcon, options);   
  elseif solver == "patternsearch"
    [mpc_params,~] = patternsearch(h_MPC, ini_params, A, b, Aeq, beq, lb, ub, nonlcon, options);   
  elseif solver == "particleswarm"
    [mpc_params,~] = particleswarm(h_MPC, nvars, lb, ub, options);   
  end
  
  % Update FIS Parameters
  range = 1;
  for a=1:n_a
    fis_params  = mpc_params(range:range+3);
    fisArray(a).Outputs.MembershipFunctions.Parameters = fis_params;
    range       = range + 4;
  end
  
  % Update initial guess
  ini_params = mpc_params;
  
  % Record new parameters
  fis_param_hist = [fis_param_hist; mpc_params(1:n_a*4)];

end
