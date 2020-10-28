% Simulate UAV system
% Date:     02/06/2020
% Author:   Craig Maxwell

% To do:
% Improve calculation of agent times - linear so can increase step size
% Problem when timestep is too large and multiple actions can be carried out?
% What happens if UAV task = 3, unable to do any other task?

%% Change Log
% 22/06/2020 - reworked UAV task assignment
% 22/06/2020 - corrected UAV distance calculations
% 22/06/2020 - improved readability

%% Bugs
% - UAVs beging assigned cells outside of search range

function [  m_s, UAV_loc, UAV_loc_hist, UAV_task, UAV_target, ...
            t_travel_UAV, t_scan_UAV] ...
            = uavModel( n_UAV, ...
                m_t_scan, m_s, UAV_loc, UAV_loc_hist, UAV_task, UAV_target, ...
                t_travel_UAV, t_scan_UAV, ...
                l_c_s_x, l_c_s_y, v_as_UAV, v_w, ang_w, dt_a, t)

  for UAV = 1:n_UAV
    if UAV_task(UAV) == 1
      % Travel
      t_travel_UAV(UAV) = t_travel_UAV(UAV) - dt_a;
      if t_travel_UAV(UAV) <= 0
        % Set task to scan
        UAV_task(UAV)   = 2;                            
        % Update UAV location
        UAV_loc(UAV, :) = UAV_target(UAV, :, 1); 
        % Update UAV location history
        UAV_loc_hist    = [UAV_loc_hist; UAV_loc(UAV, :), UAV, t];        
        % Initialise remaining scantime
        t_scan_UAV(UAV) = m_t_scan(UAV_loc(UAV, 1), UAV_loc(UAV, 2)) + t_travel_UAV(UAV);
      end
    elseif UAV_task(UAV) == 2
      % Scan
      t_scan_UAV(UAV) = t_scan_UAV(UAV) - dt_a;
      if t_scan_UAV(UAV) <= 0
        % Set task to travel
        UAV_task(UAV) = 1;
        % Set cell to scanned
        m_s(UAV_loc(UAV, 1), UAV_loc(UAV, 2)) = 1; 
        
        % Shift target list
        UAV_target(UAV, 1, :)   = circshift(UAV_target(UAV, 1, :), -1);
        UAV_target(UAV, 2, :)   = circshift(UAV_target(UAV, 2, :), -1);        
        UAV_target(UAV, :, end) = [NaN, NaN];
       
        if(isnan(UAV_target(UAV, :, 1)))
          % Set task to idle
          UAV_task(UAV) = 3;
        else
          % Calculate travel time
          t_travel_UAV(UAV) = travelTime(UAV_loc(UAV, :), UAV_target(UAV, :, 1), ...
            l_c_s_x, l_c_s_y, ang_w, v_w, v_as_UAV);
        end
      end
    elseif (UAV_task == 3)
      return
    end
  end
end
