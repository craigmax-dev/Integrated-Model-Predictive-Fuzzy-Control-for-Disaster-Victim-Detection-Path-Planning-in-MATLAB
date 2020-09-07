%% Change Log
% 01/08/2020 - removed use of n_x_search and n_y_search

%% Bugs

function [UAV_target, m_scan_schedule] = taskAssignment(...
  UAV, UAV_target, q, m_att, m_scan_schedule)
  if(sum(isnan(m_att), 'all') == numel(m_att))
    UAV_target(UAV, :, q) = NaN;
  else
    % Find max attraction cells
    [row, col] = find(m_att == max(m_att, [], 'all'));
    % Assign first max attraction in case of duplicates
    UAV_target(UAV, :, q) = [row(1), col(1)];
    % Update scan schedule map
    m_scan_schedule(row(1), col(1)) = 1;
  end
end
