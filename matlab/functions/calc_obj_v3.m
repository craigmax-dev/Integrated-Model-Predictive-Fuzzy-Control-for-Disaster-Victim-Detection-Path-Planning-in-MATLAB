%% Function objEval
% Evaluate objective function

% V2
function [s_obj, obj] = calc_obj_v3(weight, m_dw, m_bo_s, m_scan, m_victim_s, dt_s, s_obj, c_f_s)

  % Coarsen environment maps to agent resolution
  m_dw_s = func_coarsen(m_dw, c_f_s); 

% Victim parameter: Use victim map if available, else use building occupancy as a proxy
  if isempty(m_victim_s)
      m_victim_s = m_bo_s;  % Proxy for victim likelihood
  end

  % m_prior_first_scan = double(m_scan == 0) .* weight.first_scan;
  m_prior_fire = (1 - m_dw_s) .* weight.fire;
  m_prior_certainty = (1 - m_scan) .* weight.repeat_scan;

  m_prior = m_victim_s .* (m_prior_certainty + m_prior_fire);

  % Ensure the objective function is always positive
  obj = sum(m_prior, 'all') * dt_s;
  s_obj = s_obj + obj; % Sum of objective over time
end
