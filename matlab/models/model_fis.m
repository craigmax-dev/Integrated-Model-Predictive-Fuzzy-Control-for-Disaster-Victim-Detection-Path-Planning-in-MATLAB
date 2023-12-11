% Simulate UAV fuzzy controller
% Date:     02/06/2020
% Author:   Craig Maxwell


% TODO
% Rename: model_fis

function  [a_target, fis_data] = model_pathPlanning( ...
            n_a, a_target, n_q, ...
            n_x_s, n_y_s, l_x_s, l_y_s, ...
            m_scan, m_t_scan, m_dw, m_prior, ...
            fisArray, ...
            a_t_trav, a_t_scan, ...
            ang_w, v_as, v_w, test_fis_sensitivity, fis_data)
  
  % Initialise maps
  m_schedule   = zeros(n_x_s, n_y_s);
  for a=1:n_a
    if ~isnan(a_target(a, :, 1))
      m_schedule(a_target(a, 1, 1), a_target(a, 2, 1)) = 1;
    end
  end
  
  % Assign tasks
	for q = 2:n_q 
    
    % Calculate normalised response time for all agents
    [m_t_response] = calc_t_response(...
      n_x_s, n_y_s, l_x_s, l_y_s, ...
      n_a, a_t_scan, a_t_trav, a_target, q, ...
      ang_w, v_w, v_as, m_t_scan);
    
    for a=1:n_a
      % FIS of agent
      fis = fisArray(a);
      % Generate attraction map
      m_att = zeros(n_x_s, n_y_s);
      
      for i=1:n_x_s
        for j=1:n_y_s
          m_att(i,j) = calc_att (fis, ...
            m_t_response(i,j,a), m_prior(i,j), m_dw(i,j), ...
            m_scan(i,j), m_schedule(i,j));
          if test_fis_sensitivity
            % Record fis data
            fis_data = [fis_data; m_t_response(i,j,a), m_prior(i,j), m_dw(i,j), m_att(i,j)];
          end
        end
      end
      
      % Task assignment
    [a_target, m_schedule] =  func_taskAssignment(...
      a, q, a_target, m_att, m_schedule);
    end
  end
end
