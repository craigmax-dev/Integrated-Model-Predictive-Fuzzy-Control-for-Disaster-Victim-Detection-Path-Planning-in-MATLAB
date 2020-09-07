%% Notes
% Should be for p = 1:q-1 or or p = 1:q?
% m_t_scan(i,j) should be included for every cell? - yes. t_nextcell is a
% measurement of time until the cell can be scanned
% Think selection of loc_1 and loc_2 is wrong

%% To do
% Test for q > 2

%% Change log
% 08/08/2020 - bugfix - loc_2 assignment

function [m_t_nextcell] = timeToNextCell(...
          n_x_search, n_y_search, l_c_s_x, l_c_s_y, ...
          n_UAV, t_scan_UAV, t_travel_UAV, UAV_target, q, ...
          ang_w, v_w, v_as_UAV, ...
          m_scan, m_scan_schedule, m_t_scan)

  % Initialise t_nextcell_mat
  m_t_nextcell = NaN(n_x_search, n_y_search, n_UAV);
  for UAV=1:n_UAV
    % Input matrix generation
    for i=1:n_x_search
      for j=1:n_y_search
        % Negative attraction for scanned or scheduled cells
        if ~(m_scan(i,j) == 1 || m_scan_schedule(i,j) == 1)
          % Travel time for current task
          t_nextcell = t_travel_UAV(UAV) + t_scan_UAV(UAV);
          % Travel time for tasks in queue
          for p = 1:q-1
            % Start location
            loc_1 = UAV_target(UAV, :, p);  
            % End location            
            if p ~= q-1
              loc_2 = UAV_target(UAV, :, p+1);
            else
              loc_2 = [i, j];
            end
            % Travel time
            [t_travel] = travelTime(loc_1, loc_2, l_c_s_x, l_c_s_y, ...
              ang_w, v_w, v_as_UAV);
            % Add to travel time
            t_nextcell  = t_nextcell + t_travel + m_t_scan(i,j);
          end
          % Add to travel time matrix
          m_t_nextcell(i, j, UAV) = t_nextcell;
       end
      end
    end
  end
  % Normalise time input in range [0 1]
  m_t_nextcell = m_t_nextcell/max(m_t_nextcell, [], 'all');
end
