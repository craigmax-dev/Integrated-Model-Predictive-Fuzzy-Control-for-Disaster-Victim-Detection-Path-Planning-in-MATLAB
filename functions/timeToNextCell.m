function [m_t_response] = timeToNextCell(...
          n_x_s, n_y_s, l_x_s, l_y_s, ...
          n_a, a_t_scan, a_t_trav, a_target, q, ...
          ang_w, v_w, v_as, m_t_scan)

  % Initialise t_nextcell_mat
  m_t_response = NaN(n_x_s, n_y_s, n_a);
  for a=1:n_a
    % Input matrix generation
    for i=1:n_x_s
      for j=1:n_y_s
        % Travel time for current task
        t_nextcell = a_t_trav(a) + a_t_scan(a);
        % Travel time for tasks in queue
        for p = 1:q-1
          % Start location
          loc_1 = a_target(a, :, p);  
          % End location            
          if p ~= q-1
            loc_2 = a_target(a, :, p+1);
          else
            loc_2 = [i, j];
          end
          % Travel time
          [t_travel] = travelTime(loc_1, loc_2, l_x_s, l_y_s, ...
            ang_w, v_w, v_as);
          % Add to travel time
          t_nextcell  = t_nextcell + t_travel + m_t_scan(i,j);
        end
        % Add to travel time matrix
        m_t_response(i, j, a) = t_nextcell;
      end
    end
  end
  % Normalise time input in range [0 1]
  m_t_response = m_t_response/max(m_t_response, [], 'all');
end
