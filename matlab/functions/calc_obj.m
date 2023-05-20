%% Function objEval
% Evaluate objective function

function [s_obj, obj] = calc_obj(...
            m_f, m_bo, m_scan, ...
            r_bo, r_fo, dt_s, s_obj, ...
            n_x_e, n_y_e, n_x_s, n_y_s, c_f_s, ...
            test_obj_sensitivity)

    % Generate active fire map
    m_fo          = m_f;
    m_fo(m_fo==1) = 0;
    m_fo(m_fo==2) = 0;
    m_fo(m_fo==3) = 1;
    m_fo(m_fo==4) = 0;
    
    % Convert scan map to environment map resolution
    m_scan_env = zeros(n_x_e, n_y_e);
    for i=1:n_x_s
      for j=1:n_y_s
        if m_scan(i,j) == 1
          % Calculate range in m_scan_inv_env
          x_min = c_f_s*(i-1) + 1;
          x_max = x_min + c_f_s - 1;
          y_min = c_f_s*(j-1) + 1;
          y_max = y_min + c_f_s - 1;
          % Set range as scanned
          m_scan_env(x_min:x_max,y_min:y_max) = 1;
        end
      end
    end
    % Inverse scan map
    m_scan_env_inv = ones(n_x_e, n_y_e) - m_scan_env;
    % Priority due to building occupancy
    m_P_bo = r_bo.*m_bo;
    % Priority due to fire occupancy
    m_P_fo = r_fo.*m_fo;    
    % Priority map
    m_P   = m_P_bo + m_P_fo;
    % Ignore already scanned cells
    m_P   = m_P.*m_scan_env_inv;
    % Objective function for current timestep
    obj   = sum(m_P, 'all')*dt_s;       
    % Sum of objective over time
    s_obj = s_obj + obj;
    % Add to obj_fun_scaling
%     if test_obj_sensitivity    
%       test_obj_sensitivity = [test_obj_sensitivity; [m_P_bo, m_P_fo]];      
%     end
end