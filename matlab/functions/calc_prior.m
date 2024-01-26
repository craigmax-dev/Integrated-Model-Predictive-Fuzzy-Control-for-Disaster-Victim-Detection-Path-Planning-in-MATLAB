% CHANGELOG
% Refactored for objective function refactor: added first scan and rescan
% component, optional victim map, 

% TODO
% Make floating point calculation
% Does this need to be per agent or can it be general? - can be done locally
% possibly

% V2 - refactor for cell re-scan
function m_prior = calc_prior(m_bo, m_dw, m_scan, t, weight, m_victim)

  % Check if m_victim is provided, else use m_bo
  if nargin < 7
    m_victim = m_bo;
  end

  m_bo = double(m_bo);
  m_dw = double(m_dw);
  m_scan = double(m_scan);
  m_victim = double(m_victim);

  % First-time scan priority
  % Using building occupancy as a basis for unscanned cells
  m_P_first_scan = double(m_scan == 0) .* m_bo * weight.first_scan;

  % Re-scan priority
  % Using victim map (if scanned) or building occupancy (if not scanned)
  time_since_scan = max(t - m_scan, 0);  % Ensuring no negative values    
  m_P_re_scan = m_victim .* weight.repeat_scan .* time_since_scan;
  % m_P_re_scan = double(m_scan ~= 0) .* m_victim .* weight.repeat_scan .* time_since_scan;

  % Incorporate downwind map and travel time
  % Assuming proximity to fire (lower m_dw) increases priority, and longer travel time decreases priority
  m_P_dw = (1 - m_dw) * weight.dw;  % Higher priority for cells closer to fire

  % Calculate overall priority
  m_prior = m_P_first_scan + m_P_re_scan + m_P_dw;
end