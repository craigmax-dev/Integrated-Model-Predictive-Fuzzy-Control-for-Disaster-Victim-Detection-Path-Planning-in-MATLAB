%% Function mpcModel
% Simulation of system over a defined horizon. Returns sum of objective function
% over horizon.

% V2

% CHANGELOG

% TODO
% Update function inputs & calls: remove r_bo, r_fo
% Update function inputs & calls: add m_victim_scan

function [s_obj_pred] ...
          = func_MPC_model(params, ...
          fisArray, test_fis_sensitivity, ...
          m_f, m_bo, m_bt, m_prior, m_s, m_scan, m_t_scan, m_victim_scan, ...
          dk_a, dk_c, dk_e, dk_mpc, dt_s, k, seed, ...
          n_a, n_MF_out, n_p, n_x_s, n_y_s, n_x_e, n_y_e, n_q, ...
          a_loc, a_target, a_task, a_t_trav, a_t_scan, ...
          l_x_s, l_y_s, c_f_s, ...
          c_fs_1, c_fs_2, v_as, v_w, ang_w, ...
          r_bo, r_fo, fis_data, config)
  flag_mpc = true;
    
  %% Variables
  % Counters
  k_pred  = 0;
  k_a     = 0;
  k_c     = 0;
  k_e     = 0;
  k_mpc   = 0;
  s_obj_pred   = 0;
  
  dt_a    = dk_a*dt_s;
  dt_e    = dk_e*dt_s;
  
  % Maps
  % Downwind map
  m_t_dw    = zeros(n_x_s, n_x_s);
  % Range
  range = 1;
  
  %% Prediction
  while k_pred < dk_mpc*n_p
    
    %% Update FIS parameters
    if k_mpc*dk_mpc <= k_pred
      for a = 1:n_a
        for mf = 1:n_MF_out
          newParams = params(range:range+3);
          fisArray(a).Outputs.MembershipFunctions(mf).Parameters = newParams;
          range = range + 4;
        end
      end
      k_mpc = k_mpc + 1;
    end
    
    %% Path planning
    if k_c*dk_c <= k_pred
      [a_target, fis_data] = model_fis( ...
            n_a, a_target, n_q, ...
            n_x_s, n_y_s, l_x_s, l_y_s, ...
            m_scan, m_t_scan, m_t_dw, m_prior, ...
            fisArray, ...
            a_t_trav, a_t_scan, ...
            ang_w, v_as, v_w, test_fis_sensitivity, fis_data);
      k_c = k_c + 1;
    end
    
    %% Agent actions
    if k_a*dk_a <= k_pred
      [  m_scan, ~, a_loc, ~, a_task, a_target, ...
            a_t_trav, a_t_scan] ...
            = model_agent( n_a, ...
                m_t_scan, m_scan, [], a_loc, [], a_task, a_target, ...
                a_t_trav, a_t_scan, ...
                l_x_s, l_y_s, v_as, v_w, ang_w, dt_a, k, flag_mpc, config);
      k_a = k_a + 1;
    end
    
    %% Environment model
    if k_e*dk_e <= k_pred
      % Update firemap and downwind map
      [m_f, m_bt, m_t_dw] = model_environment(...
        m_f, m_s, m_bo, m_bt, dt_e, k, seed, n_x_e, n_y_e, ...
        v_w, ang_w, c_fs_1, c_fs_2, c_f_s, flag_mpc);
      k_e = k_e + 1;
    end
    
    %% Objective function evaluation
    % V1
%     [s_obj_pred, ~] = calc_obj(...
%             m_f, m_bo, m_scan, ...
%             r_bo, r_fo, dt_s, s_obj_pred, ...
%             n_x_e, n_y_e, c_f_s, config);

    [s_obj_pred, ~] = calc_obj(...
            config, m_f, m_bo, m_scan, m_victim_scan, ...
            dt_s, s_obj_pred, n_x_e, n_y_e, c_f_s);      
          
    %% Advance timestep
    k_pred = k_pred + 1;
    k      = k + 1;
  end
end
