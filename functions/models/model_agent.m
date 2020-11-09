% Simulate agent system
% Date:     02/06/2020
% Author:   Craig Maxwell

function [  m_scan, m_scan_hist, a_loc, a_loc_hist, a_task, a_target, ...
            a_t_trav, a_t_scan] ...
            = model_agent( n_a, ...
                m_t_scan, m_scan, m_scan_hist, a_loc, a_loc_hist, a_task, a_target, ...
                a_t_trav, a_t_scan, ...
                l_x_s, l_y_s, v_as, v_w, ang_w, dt_a, k, flag_mpc)
  for a = 1:n_a
    if a_task(a) == 1
      % Travel
      a_t_trav(a) = a_t_trav(a) - dt_a;
      if a_t_trav(a) <= 0
        % Set task to scan
        a_task(a)   = 2;                            
        % Update agent location
        a_loc(a, :) = a_target(a, :, 1);
        % Update agent location history
        a_loc_hist    = [a_loc_hist; a_loc(a, :), a, k];        
        % Initialise remaining scantime, using additional time from the travel
        a_t_scan(a) = m_t_scan(a_loc(a, 1), a_loc(a, 2)) + a_t_trav(a);
      end
    elseif a_task(a) == 2
      % Scan
      a_t_scan(a) = a_t_scan(a) - dt_a;
      if a_t_scan(a) <= 0
        % Set task to travel
        a_task(a) = 1;
        % Get cell coordinates
        i = a_loc(a, 1);
        j = a_loc(a, 2);
        % Set cell to scanned
        m_scan(i,j) = 1;
        % Update scan history
        if ~flag_mpc        
          m_scan_hist(i,j) = k;
        end
        % Shift target list
        a_target(a, 1, :)   = circshift(a_target(a, 1, :), -1);
        a_target(a, 2, :)   = circshift(a_target(a, 2, :), -1);        
        a_target(a, :, end) = [NaN, NaN];
        % Set task to idle if no target, otherwise calculate travel time
        if(isnan(a_target(a, :, 1)))
          a_task(a) = 3;
        else
          a_t_trav(a) = calc_t_trav(a_loc(a, :), a_target(a, :, 1), ...
            l_x_s, l_y_s, ang_w, v_w, v_as);
        end
      end
    elseif (a_task == 3)
      return
    end
  end
end