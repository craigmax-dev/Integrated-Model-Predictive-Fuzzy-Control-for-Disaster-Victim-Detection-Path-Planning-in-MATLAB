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

function  [a_target, fis_data] = pathPlanner( ...
            n_a, a_target, l_queue, ...
            n_x_s, n_y_s, l_x_s, l_x_y, ...
            m_scan, m_t_scan, m_t_dw, m_prior, ...
            fisArray, ...
            a_t_trav, a_t_scan, ...
            ang_w, v_as, v_w, test_fis_sensitivity, fis_data)
  
  % Initialise maps
  m_att        = NaN(n_x_s, n_y_s, n_a);
  m_schedule   = zeros(n_x_s, n_y_s);
  for a=1:n_a
    if ~isnan(a_target(a, 1, 1))
      m_schedule([a_target(a, 1, 1), a_target(a, 2, 1)]) = 1;      
    end
  end
  
  % Assign tasks
	for q = 2:l_queue
      % Generate t_nextcell_mat      
      [m_t_response] = timeToNextCell(...
        n_x_s, n_y_s, l_x_s, l_x_y, ...
        n_a, a_t_scan, a_t_trav, a_target, q, ...
        ang_w, v_w, v_as, m_t_scan);
    for a=1:n_a
      % Generate attraction map   
      hdl_attcalc = @(scan_state, schedule_state, t_response, prior, t_dw) attcalc (fisArray(a), scan_state, schedule_state, t_response, prior, t_dw);
      m_att(:,:,a) = arrayfun(hdl_attcalc, m_scan, m_schedule, m_t_response(:,:,a), m_prior, m_t_dw);
      % Record fis data
      if test_fis_sensitivity
        for i=1:n_x_s
          for j=1:n_y_s
            fis_data = [fis_data; m_t_response(i,j), m_prior(i,j), m_t_dw(i,j), m_att(i,j)];
          end
        end        
      end
      % Task assignment
      [a_target, m_schedule] = taskAssignment(...
          a, a_target, q, m_att(:,:,a), m_schedule);        
    end
%     %% Check for conflicting targets and reschedule
%     % Determine conflicting targets
%     while size(a_target(:,:,q),1) ~= size(unique(a_target(:,:,q), 'rows'),1)
%       fprintf("Reschedule")
%       for UAV_1=1:n_a
%         for UAV_2=1:n_a
%           if UAV_1 ~= UAV_2
%             UAV_1_target = a_target(UAV_1, :, q);
%             UAV_2_target = a_target(UAV_2, :, q);
%             if isequal(UAV_1_target, UAV_2_target)
%               % Reassign UAV with lower attraction
%               if m_att(UAV_1_target(1), UAV_1_target(2), UAV_1) >=  m_att(UAV_2_target(1), UAV_2_target(2), UAV_2)
%                 % Set attraction negative
%                 m_att(UAV_2_target(1), UAV_2_target(2), UAV_2) = negAtt;
%                 % Task assignment
%                 [a_target, m_schedule] = taskAssignment(...
%                   UAV_2, a_target, q, m_att(:,:,UAV), m_schedule);
%               else
%                 % Set attraction negative
%                 m_att(UAV_1_target(1), UAV_1_target(2), UAV_1) = negAtt;               
%                 % Task assignment
%                 [a_target, m_schedule] = taskAssignment(...
%                   UAV_1, a_target, q, m_att(:,:,UAV), m_schedule);
%               end
%             end
%           end        
%         end
%       end
%     end
  end
end