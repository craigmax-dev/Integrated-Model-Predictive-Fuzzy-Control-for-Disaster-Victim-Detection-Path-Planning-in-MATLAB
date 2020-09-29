%% Function mpcModel
% Simulation of system over a defined horizon. Returns sum of objective function
% over horizon.

function [s_obj_pred] ...
          = mpcModel( params, ...
          fisArray, test_fis_sensitivity, ...
          m_f, m_f_hist, m_r, m_bo, m_bt, m_scan, m_t_scan, ...
          dk_a, dk_c, dk_e, dk_mpc, dt_s, k, ...
          n_a, n_p, n_x_s, n_y_s, n_x_f, n_y_f, l_q, ...
          a_loc, a_target, a_task, a_t_trav, a_t_scan, ...
          l_c_s_x, l_c_s_y, c_f_search, ...
          c_w1, c_w2, v_as, v_w, ang_w, ...
          r_bo, r_f)
        
  %% Variables
  % Counters
  k_pred  = 0;
  k_a     = 0;
  k_c     = 0;
  k_e     = 0;
  s_obj_pred  = 0;
  ct_mpc_pred = 0;
  
  dt_a    = dk_a*dt_s;
  dt_c    = dk_c*dt_s;
  dt_e    = dk_e*dt_s;
  dt_mpc  = dk_mpc*dt_s;
  dt_prog = dk_prog*dt_s;
  
  % Maps
  % Downwind map
  m_dw    = zeros(n_x_s, n_x_s);
  % Priority map 
  m_prior = zeros(n_x_s, n_y_s);
    
  %% Prediction
  while k_pred <= dk_mpc*n_p
    
    %% Update FIS parameters
    if ct_mpc_pred*dk_mpc < k_pred
      for UAV=1:n_a
        range = 1 + (ct_mpc_pred * n_a * 4) + (UAV - 1) * 4;
        newParams = params(range:range+3);
        fisArray(UAV).Outputs.MembershipFunctions.Parameters = newParams;
      end
      ct_mpc_pred = ct_mpc_pred + 1;
    end
    %% Path planning
    if k_c*dk_c <= k_pred
      [a_target] = pathPlanner(n_a, a_target, l_q, ...
                      n_x_s, n_y_s, l_c_s_x, l_c_s_y, ...
                      m_scan, m_t_scan, m_dw, m_prior, ...
                      fisArray, ...
                      a_t_trav, a_t_scan, ...
                      ang_w, v_as, v_w, test_fis_sensitivity);
      k_c = k_c + 1;
    end
    %% Agent actions
    if k_a*dk_a <= k_pred
      [   m_scan, ~, a_loc, ~, a_task, a_target, ...
          a_t_trav, a_t_scan] ...
                      = agentModel( n_a, ...
                      m_t_scan, m_scan, [], ...
                      a_loc, [], a_task, a_target, ...
                      a_t_trav, a_t_scan, ...
                      l_c_s_x, l_c_s_y, v_as, v_w, ang_w, dt_a, k_pred, true);
      k_a = k_a + 1;
    end
    %% Environment model
    if k_e*dk_e <= k_pred
      % Update firemap and downwind map
      [m_f, m_f_hist, m_bt, m_dw] = environmentModel(  m_f, m_f_hist, m_r, m_bo, m_bt, ...
                                      dt_e, k, n_x_f, n_y_f, ...
                                      v_w, ang_w, ...
                                      c_w1, c_w2, c_f_search, true);
      k_e = k_e + 1;
    end
    %% Objective function evaluation
    [s_obj_pred, ~] ...
        = objEval(m_f, m_bo, m_scan, ...
          r_bo, r_f, dt_s, s_obj_pred, ...
          n_x_f, n_y_f, n_x_s, n_y_s, c_f_search);

    %% Advance timestep
    k_pred = k_pred + dt_s;
    k      = k + 1;
  end
end