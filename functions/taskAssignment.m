%% Change Log
% 01/08/2020 - removed use of n_x_search and n_y_search

%% Bugs

function [a_target, m_scan_schedule] = taskAssignment(...
  a, a_target, q, m_att, m_scan_schedule)
  if(sum(isnan(m_att), 'all') == numel(m_att))
    a_target(a, :, q) = NaN;
  else
    % Find max attraction cells
    [i, j] = find(m_att == max(m_att, [], 'all'));
    if isnan(m_att(i, j))
      fprintf("Incorrect assignment")
    end
    if length(i) > 1 || length(j) > 1
      i = i(1);
      j = j(1);
    end
    % Assign first max attraction in case of duplicates
    a_target(a, :, q) = [i, j];
    % Update scan schedule map
    m_scan_schedule(i, j) = 1;
  end
end