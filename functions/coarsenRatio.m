%% Calculate coarsen ratio given desired cell dimensions

%% Notes
% Should round down or to nearest?

function [c_f, l_c_x, l_c_y] = coarsenRatio(ref, l_d)
    
  % Difference in lat negligible for small scale
  lat = ref.LatitudeLimits(1);
  m_per_deg_lat = 111132.954 - 559.822 * cos( 2 * lat ) + 1.175 * cos( 4 * lat);
  m_per_deg_lon = 111132.954 * cos ( lat );
  l_r_lat = ref.CellExtentInLatitude * m_per_deg_lat;
  l_r_lon = ref.CellExtentInLongitude * m_per_deg_lon;

  % Coarsen to desired cell size - side to right is removed if necessary
  n_lat   = round(l_d/l_r_lat);
  n_lon   = round(l_d/l_r_lon);
  c_f     = [n_lat, n_lon];
  % length of new cells in x direction
  l_c_x   = n_lat * m_per_deg_lat * ref.CellExtentInLatitude;  
  % length of new cells in y direction
  l_c_y   = n_lon * m_per_deg_lon * ref.CellExtentInLongitude;

  if n_lat < 2 || n_lon < 2
    error("Coarsen ratio not high enough")
  end
end
