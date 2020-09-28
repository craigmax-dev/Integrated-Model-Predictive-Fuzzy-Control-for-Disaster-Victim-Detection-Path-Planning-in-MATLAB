function [a_target, m_schedule] = taskAssignment(...
  a, q, a_target, m_att, m_schedule)
  if(sum(isnan(m_att), 'all') == numel(m_att))
    a_target(a, :, q) = NaN;
  else
    % Find max attraction cell. If multiple return first one.
    [i, j] = find(m_att == max(m_att, [], 'all'), 1);
    % Assign cell as target
    a_target(a, :, q) = [i, j];
    % Update scan schedule map
    m_schedule(i, j) = 1;
  end
end