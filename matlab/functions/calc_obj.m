%% Function objEval
% Evaluate objective function

% V2

% CHANGELOG
% - coarsen all environment maps to agent maps
% - removed inputs n_x_e, n_y_e
% - refactor: replaced flags with config weights
% - refactor: m_scan now contains time at which the cell was scanned
% - refactor: all objective function components fully configurable
% - refactor: first scan, repeat scan, downwind time, optional m_victim

% V2 REFACTOR V5
function [s_obj, obj] = calc_obj(weight, m_f, m_bo_s, m_scan, m_victim_s, dt_s, s_obj, c_f_s, t)
    
  % Active fires map
  m_fo = (m_f == 3);

  % Victim parameter: Use victim map if available, else use building occupancy as a proxy
  if isempty(m_victim_s)
      m_victim_s = m_bo_s;  % Proxy for victim likelihood
  end

  % Coarsen environment maps to agent resolution
  m_fo_s = func_coarsen(m_fo, c_f_s); 

  % First-time scan priority component
  m_P_first_scan = double(m_scan == 0) .* m_victim_s .* weight.first_scan;

  % Repeat scan priority component
  time_since_last_scan = max(t - m_scan, 0);  % Time since each cell was last scanned
  m_P_repeat_scan = time_since_last_scan .* m_victim_s .* weight.repeat_scan;

  % Priority from active or catching fire
  m_P_fire = m_victim_s .* m_fo_s .* weight.fire;

  % Total priority map
  m_P = m_P_first_scan + m_P_repeat_scan + m_P_fire;

  % Ensure the objective function is always positive
  obj = sum(m_P, 'all') * dt_s;
  s_obj = s_obj + obj; % Sum of objective over time
end
