%% coarsenRatio.m
% For a given desired cell length in a tif object, return the required 
% coarsen ratio and resulting cell lengths in both axes in order to closest
% match that cell length.

function [c_f, l_x, l_y] = calc_coarsenRatio(ref, l_d)
    
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
  l_x   = n_lat * m_per_deg_lat * ref.CellExtentInLatitude;  
  % length of new cells in y direction
  l_y   = n_lon * m_per_deg_lon * ref.CellExtentInLongitude;

  if n_lat < 2 || n_lon < 2
    error("Coarsen ratio not high enough")
  end
end