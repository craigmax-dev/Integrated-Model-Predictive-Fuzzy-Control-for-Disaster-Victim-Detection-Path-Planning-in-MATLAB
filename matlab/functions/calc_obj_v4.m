function [s_obj, obj] = calc_obj_v4(weight, m_dw, m_bo_s, m_scan, m_victim_s, dt_s, s_obj, c_f_s)

  % CALC_OBJ_V4 Calculate the objective function for the MPC framework.
  %
  % This function computes the objective function value based on the given 
  % weights, downwind risk, building occupancy, scan certainty, and victim likelihood 
  % maps. The objective function aims to minimize the risk to victims while considering 
  % the scan certainty and downwind risk.
  %
  % INPUTS:
  %   weight        - Structure containing weights for different risk factors.
  %                   Fields:
  %                       .fire         - Weight for fire risk.
  %                       .repeat_scan  - Weight for scan certainty risk.
  %   m_dw          - Downwind risk map (matrix).
  %   m_bo_s        - Building occupancy map (matrix).
  %   m_scan        - Scan certainty map (matrix).
  %   m_victim_s    - Victim likelihood map (matrix). If empty, m_bo_s is used.
  %   dt_s          - Time step duration (scalar).
  %   s_obj         - Cumulative objective function value (scalar).
  %   c_f_s         - Coarsening factor (scalar).
  %
  % OUTPUTS:
  %   s_obj         - Updated cumulative objective function value (scalar).
  %   obj           - Objective function value at the current time step (scalar).
  %
  % DESCRIPTION:
  % The function proceeds as follows:
  % 1. Coarsens the downwind risk map to match the agent resolution.
  % 2. Uses the victim likelihood map if available, otherwise uses the building 
  %    occupancy map as a proxy.
  % 3. Computes the fire risk and scan certainty risk.
  % 4. Combines the risks to calculate the overall priority map.
  % 5. Ensures the objective function value is always positive by summing the 
  %    overall priority map elements and scaling by the time step duration.
  % 6. Updates the cumulative objective function value.

  %   [s_obj, obj] = calc_obj_v4(weight, m_dw, m_bo_s, m_scan, m_victim_s, dt_s, s_obj, c_f_s);

  % Coarsen environment maps to agent resolution
  m_dw_s = func_coarsen(m_dw, c_f_s); 

% Victim parameter: Use victim map if available, else use building occupancy as a proxy
  if isempty(m_victim_s)
      m_victim_s = m_bo_s;  % Proxy for victim likelihood
  end

  % m_prior_first_scan = double(m_scan == 0) .* weight.first_scan;
  m_prior_fire = (1 - m_dw_s) .* weight.fire;
  m_prior_certainty = (1 - m_scan) .* weight.repeat_scan;

  m_prior = m_victim_s .* m_prior_certainty .* (1 + m_prior_fire);

  % Ensure the objective function is always positive
  obj = sum(m_prior, 'all') * dt_s;
  s_obj = s_obj + obj; % Sum of objective over time
end