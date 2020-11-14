%% Function initialise_plotting
% Initialise plotting

function [axis_x_e, axis_y_e, axis_x_s, axis_y_s, ...
  m_f_hist, m_f_hist_animate, m_bt_hist_animate, m_dw_hist_animate, ...
  m_scan_hist, a_loc_hist, t_hist, fis_param_hist, obj_hist, s_obj_hist] ...
  = initialise_plotting(m_p_ref, n_x_e, n_y_e, n_x_s, n_y_s, m_f, m_bt, n_a, a_loc, k, fis_params)
  % Axes may not be entirely accurate as coarsening may remove some
  % rows/columns from original map.
  % Axes for dynamic environment states
  axis_x_e = linspace(m_p_ref.LatitudeLimits(1),  m_p_ref.LatitudeLimits(2),  n_x_e);
  axis_y_e = linspace(m_p_ref.LongitudeLimits(1), m_p_ref.LongitudeLimits(2), n_y_e);
  % Axes for search map
  axis_x_s = linspace(m_p_ref.LatitudeLimits(1),  m_p_ref.LatitudeLimits(2),  n_x_s);
  axis_y_s = linspace(m_p_ref.LongitudeLimits(1), m_p_ref.LongitudeLimits(2), n_y_s);
  % History plots
  obj_hist    = [];
  s_obj_hist  = [];
  t_hist      = [];
  m_f_hist    = m_f;
  m_f_hist_animate  = m_f;
  m_bt_hist_animate = m_bt;
  a_loc_hist    = [];
  for a = 1:n_a
    a_loc_hist(a,:) = [a_loc(a, 1), a_loc(a, 2), a, k];
  end
  m_scan_hist       = zeros(n_x_s, n_y_s);
  m_dw_hist_animate = zeros(n_x_s, n_y_s);
  fis_param_hist    = fis_params;
end