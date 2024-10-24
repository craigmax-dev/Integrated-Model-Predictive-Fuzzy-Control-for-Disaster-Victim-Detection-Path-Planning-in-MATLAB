% CHANGELOG
% Refactored for objective function refactor: added first scan and rescan
% component, optional victim map
% Bugfix: fixed treatment of victim map

% V2.2 - added victim model flag
function m_prior = calc_prior(m_bo, m_dw, m_scan, t, weight, m_victim, flag_victim_model)
  
  % Restrict to range 0-1 to prevent too high attraction
  if flag_victim_model
    m_victim = m_victim ./ max(m_victim, [], 'all');
  end

  % Convert matrices to double for calculation
  m_bo = double(m_bo);
  m_dw = double(m_dw);
  m_scan = double(m_scan);
  m_victim = double(m_victim);

  % First-time scan priority
  % Using building occupancy as a basis for unscanned cells
  m_P_first_scan = double(m_scan == 0) .* m_bo * weight.first_scan;

  % Re-scan priority
  % Decide on the basis for re-scan priority based on flag_victim_model
  % NOTE: The second component is an option, but requires gentle balancing
  % with the first scan priority
  time_since_scan = max(t - m_scan, 0);  % Ensuring no negative values
  if flag_victim_model
    % m_P_re_scan = m_victim .* weight.repeat_scan .* time_since_scan;
    m_P_re_scan = double(m_scan ~= 0) .* m_victim .* weight.repeat_scan .* time_since_scan + 1;
  else
    % m_P_re_scan = m_bo .* weight.repeat_scan .* time_since_scan;
    m_P_re_scan = double(m_scan ~= 0) .* m_bo .* weight.repeat_scan .* time_since_scan + 1;
  end

  % Incorporate downwind map and travel time
  % Assuming proximity to fire (lower m_dw) increases priority, and longer travel time decreases priority
  m_P_dw = (1 - m_dw) * weight.dw;  % Higher priority for cells closer to fire

  % Calculate overall priority
  m_prior = m_P_first_scan + m_P_re_scan + m_P_dw;
end