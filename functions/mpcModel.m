%% Function mpcModel
% Simulation of system over a defined horizon. Returns sum of objective function
% over horizon.

%% Change Log
% 02/07/2020 - Created function
% 02/07/2020 - 
% 02/07/2020 - 

%% Notes
% Should prediction horizon be equal to the time between MPC optimsiations?
% Can hist varibales be removed?

%% To do
% Add plotting of optimisation results
% Add plotting of all FIS surfaces - https://nl.mathworks.com/help/matlab/math/plot-functions.html
% Add constraints so rules are always fired?

%% Bugs

function [s_obj_pred] ...
          = mpcModel( params, ...
          fisArray, test_fis_sensitivity, ...
          m_f, m_f_hist, m_r, m_bo, m_bt, m_scan, m_t_scan, ...
          dt_a, dt_c, dt_f, dt_mpc, dt_s,  ...
          n_a, n_p, n_x_s, n_y_s, n_x_f, n_y_f, l_q, ...
          a_loc, a_target, a_task, a_t_trav, a_t_scan, ...
          k, negAtt, ...
          l_c_s_x, l_c_s_y, c_f_search, ...
          c_w1, c_w2, v_as, v_w, ang_w, ...
          r_bo, r_f)
        
  %% Variables
  % Counters
  t_pred      = 0;
  ct_a_pred   = 0;
  ct_c_pred   = 0;
  ct_f_pred   = 0;
  s_obj_pred  = 0;
  ct_mpc_pred = 0;
  
  % Maps
  % Downwind map
  m_dw    = zeros(n_x_s, n_x_s);
  % Priority map 
  m_prior = zeros(n_x_s, n_y_s);
  
  % Unused variables - necessary for simulation to run
  % Scan map history
  m_scan_hist = zeros(n_x_s, n_y_s);
  % Scan map history times
  m_scan_t_hist = [];
  % UAV location history
  UAV_loc_hist  = zeros(1, 4);              % [loc(1), loc(2), UAV, t]
    
  %% Prediction
  while t_pred <= dt_mpc*n_p
    
    %% Update FIS parameters
    if ct_mpc_pred*dt_mpc < t_pred
      for UAV=1:n_a
        range = 1 + (ct_mpc_pred * n_a * 4) + (UAV - 1) * 4;
        newParams = params(range:range+3);
        fisArray(UAV).Outputs.MembershipFunctions.Parameters = newParams;
      end
      ct_mpc_pred = ct_mpc_pred + 1;
    end
    %% Path planning
    if ct_c_pred*dt_c <= t_pred
      [a_target] = pathPlanner(n_a, a_target, l_q, ...
                      n_x_s, n_y_s, l_c_s_x, l_c_s_y, ...
                      m_scan, m_t_scan, m_dw, m_prior, ...
                      fisArray, ...
                      a_t_trav, a_t_scan, ...
                      ang_w, v_as, v_w, test_fis_sensitivity);
      ct_c_pred = ct_c_pred + 1;
    end
    %% Agent actions
    if ct_a_pred*dt_a <= t_pred
      [   m_scan, m_scan_hist, a_loc, UAV_loc_hist, a_task, a_target, ...
          a_t_trav, ~, a_t_scan] ...
                      = agentModel( n_a, ...
                      m_t_scan, m_scan, m_scan_hist, ...
                      a_loc, UAV_loc_hist, a_task, a_target, ...
                      a_t_trav, [], a_t_scan, ...
                      l_c_s_x, l_c_s_y, v_as, v_w, ang_w, dt_a, t_pred, true);
      ct_a_pred = ct_a_pred + 1;
    end
    %% Environment model
    if ct_f_pred*dt_f <= t_pred
      % Update firemap and downwind map
      [m_f, m_f_hist, m_bt, m_dw] = environmentModel(  m_f, m_f_hist, m_r, m_bo, m_bt, ...
                                      dt_f, k, n_x_f, n_y_f, ...
                                      v_w, ang_w, ...
                                      c_w1, c_w2, c_f_search, true);
      ct_f_pred = ct_f_pred + 1;
    end
    %% Objective function evaluation
    [s_obj_pred, ~] ...
        = objEval(m_f, m_bo, m_scan, ...
          r_bo, r_f, dt_s, s_obj_pred, ...
          n_x_f, n_y_f, n_x_s, n_y_s, c_f_search);

    %% Advance timestep
    t_pred = t_pred + dt_s;
    k      = k + 1;
  end
end