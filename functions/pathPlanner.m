% Simulate UAV fuzzy controller
% Date:     02/06/2020
% Author:   Craig Maxwell

%% Change log
% 22/06/2020 - Removed fireModel evaluation from script
% 22/06/2020 - Removed UAV_occ flag - instead "NaN" can be used inside UAV tasks
% 22/06/2020 - Fixed confusing indexing in travel time calculation
% 22/06/2020 - Fixed travel time calculation
% 22/06/2020 - Expanded travel time calculation for any value of l_queue
% 22/06/2020 - Travel times saved after calculation
% 29/06/2020 - Bugfix - problems with path planning when entire environment
% scanned.
% 01/07/2020 - Added empty attraction map consideration to conflict resolution
% script
% 27/07/2020 - Normalisation of time input - same for all UAVs but can change in
% queue
% 28/07/2020 - Simplified skip criteria
% 30/07/2020 - bugfix - m_scan_schedule and t_nextcell_mat handling
% 30/07/2020 - bugfix - schedule conflict execution condition
% 30/07/2020 - syntax fix - replace nested sum function with sum(mat,'all')
% 30/07/2020 - bugfix - task reassignment algorithm using wrong indices in if
% statement
% 30/07/2020 - syntax fix - use max(mat,[],'all') instead of nested max
% 30/07/2020 - created task assignment function
% 30/07/2020 - created attraction function
% 30/07/2020 - created timeToNextCell function

%% Notes

%% To do 
% Better method for normalisation

%% Bugs

function  [UAV_target, fis_var_scaling] = pathPlanner( ...
            n_UAV, UAV_target, l_queue, ...
            n_x_search, n_y_search, l_c_s_x, l_c_s_y, ...
            m_scan, m_t_scan, m_dw, m_prior, ...
            fisArray, ...
            t_travel_UAV, t_scan_UAV, ...
            ang_w, v_as_UAV, v_w, ...
            negAtt, fis_var_scaling)
  
  % Initialise maps
  m_att             = NaN(n_x_search, n_y_search, n_UAV);
  m_scan_schedule   = zeros(n_x_search, n_y_search);
  for UAV=1:n_UAV
    if ~isnan(UAV_target(UAV, 1, 1))
      m_scan_schedule([UAV_target(UAV, 1, 1), UAV_target(UAV, 2, 1)]) = 1;      
    end
  end
  
  % Assign tasks
	for q = 2:l_queue
      % Generate t_nextcell_mat      
      [m_t_nextcell] = timeToNextCell(...
        n_x_search, n_y_search, l_c_s_x, l_c_s_y, ...
        n_UAV, t_scan_UAV, t_travel_UAV, UAV_target, q, ...
        ang_w, v_w, v_as_UAV, ...
        m_scan, m_scan_schedule, m_t_scan);
    for UAV=1:n_UAV
      % Generate attraction map   
      hdl_attcalc = @(t_nextcell, prior, t_dw) attcalc (fisArray(UAV), t_nextcell, prior, t_dw);
      m_att(:,:,UAV) = arrayfun(hdl_attcalc, m_t_nextcell(:,:,UAV), m_prior, m_dw);
      % Task assignment
      [UAV_target, m_scan_schedule] = taskAssignment(...
          UAV, UAV_target, q, m_att(:,:,UAV), m_scan_schedule);        
    end

    %% Check for conflicting targets and reschedule
    % Determine conflicting targets
    while size(UAV_target(:,:,q),1) ~= size(unique(UAV_target(:,:,q), 'rows'),1)
      for UAV_1=1:n_UAV
        for UAV_2=1:n_UAV
          if UAV_1 ~= UAV_2
            UAV_1_target = UAV_target(UAV_1, :, q);
            UAV_2_target = UAV_target(UAV_2, :, q);
            if isequal(UAV_1_target, UAV_2_target)
              % Reassign UAV with lower attraction
              if m_att(UAV_1_target(1), UAV_1_target(2), UAV_1) >=  m_att(UAV_2_target(1), UAV_2_target(2), UAV_2)
                % Set attraction negative
                m_att(UAV_2_target(1), UAV_2_target(2), UAV_2) = negAtt;
                % Task assignment
                [UAV_target, m_scan_schedule] = taskAssignment(...
                  UAV_2, UAV_target, q, m_att(:,:,UAV), m_scan_schedule);
              else
                % Set attraction negative
                m_att(UAV_1_target(1), UAV_1_target(2), UAV_1) = negAtt;               
                % Task assignment
                [UAV_target, m_scan_schedule] = taskAssignment(...
                  UAV_1, UAV_target, q, m_att(:,:,UAV), m_scan_schedule);
              end
            end
          end        
        end
      end
    end
  end
end
