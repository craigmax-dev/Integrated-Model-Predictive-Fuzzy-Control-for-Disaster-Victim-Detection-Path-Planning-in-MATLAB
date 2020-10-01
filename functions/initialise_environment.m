%% Function initialise_environment
% Initialise environment model

function [l_x_e, l_y_e, n_x_e, n_y_e, ...
  m_bo, m_s, m_f, m_bt, m_dw, m_p_ref, ...
  c_fs_1, c_fs_2, c_f_s, v_w, ang_w] = initialise_environment(dt_e, k)
  % Import building raster
  buildingRaster      = 'data\maps\portAuPrince\portAuPrince_campeche.tif';
  [m_p_in, m_p_ref]   = geotiffread(buildingRaster);
  m_p_in              = m_p_in(50:450, 50:250);
  % Environment map cell length (m)
  l_c_e   = 3;
  % Building occupancy map and structure map
  [c_f_e, l_x_e, l_y_e] = calc_coarsenRatio(m_p_ref, l_c_e);
  [m_bo, m_s] = func_coarsen(m_p_in, c_f_e);
  % Environment map dimensions
  n_x_e       = size(m_bo,1);
  n_y_e       = size(m_bo,2);
  % Wind model
  v_w         = 2;        % Wind speed (m/s)
  ang_w       = pi/2;     % Wind direction (rad) - [-pi/2 pi/2] - w_d = 0 in +ve y axis
  % Initialise fire map
  m_f       = m_s;
  m_f(80:82,1:2) = 3 * m_s(80:82,1:2);
  % Initialise burntime map
  m_bt        = zeros(n_x_e,n_y_e);
  % Fire model parameters
  c_fs_1        = 0.2;    % Wind constant 1 (for fire model)
  c_fs_2        = 0.2;    % Wind constant 2 (for fire model)
  c_f_s         = [5, 5];
  %  Initialise fire maps
  [m_f, ~, ~, ~, ...
  m_bt, m_dw] = model_environment(...
  m_f, [], [], [], m_s, m_bo, m_bt, ...
  dt_e, k, n_x_e, n_y_e, v_w, ang_w, c_fs_1, c_fs_2, c_f_s, false);
end

