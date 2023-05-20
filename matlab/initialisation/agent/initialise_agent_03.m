%% Function initialise_agent
% Initialise agent model

function [n_x_s, n_y_s, l_x_s, l_y_s, n_a, n_q, v_as, a_t_trav, ...
  t_scan_m, t_scan_c, a_task, a_loc, a_target, a_t_scan, ...
  m_scan, m_t_scan, m_bo_s] = initialise_agent_03(m_bo, l_x_e, l_y_e)
  % Search map coarsen factors
  c_f_s  = [5, 5];
  % Search map building occupancy
  m_bo_s = func_coarsen(m_bo, c_f_s); 
  % Search map dimensions
  n_x_s  = size(m_bo_s, 1);
  n_y_s  = size(m_bo_s, 2);
  % Search map cell lengths
  l_x_s     = c_f_s(1)*l_x_e;
  l_y_s     = c_f_s(2)*l_y_e;
  % Agent parameters
  n_a           = 3;                % Number of UAVs in simulation
  n_q           = 2;                % Queue length for UAV tasks
  v_as          = 5;                % UAV airspeed (m/s)
  a_t_trav      = zeros(n_a, 1);    % Time left to complete travel
  t_scan_m      = 0.1;              % Scan time per square metre
  a_task        = 2.*ones(n_a, 1);% Current task for UAVs
  a_loc         = [ 1, 1;
                    1, 2;
                    1, 3];          % Current locations of UAVs
  % Agent targets
  a_target        = nan(n_a, 2, n_q);
  a_target(:,:,1) = a_loc;
  % Search map cell scan time
  t_scan_c    = t_scan_m*l_x_s*l_y_s;       % Scan time per cell
  m_scan      = zeros(n_x_s, n_y_s);        % Scan map
  % Unsorted
  m_t_scan    = t_scan_c.*ones(n_x_s, n_y_s); % Scan time map (s) - time to scan each cell
  a_t_scan  = zeros(n_a, 1);    % Time left to complete current scanning task
  for a = 1:n_a
      a_t_scan(a) = m_bo_s(a_loc(a, 1), a_loc(a, 2));
  end

end