%% Coarsen raster grid
% Generate coarsened occupancy map and raster map based on input map
% Inputs:
% - m_p_in - input map
% - ref - (optional, requires l_d) raster reference map
% - l_d - (optional, requires ref) reference cell size
% - c_f - (optional) coarsening factor
% m_occ - occupancy map (percent of each cell occupied)
% m_r   - raster map (binary, identifies if any occupancy in cell)

%% Assumptions
% - some error introduced by rounding - ignoring rows and column on sides
% of matrix.

%% Coarsen raster
function [m_occ, m_r] = coarsen(m_p_in, c_f)

  n_lat = c_f(1);
  n_lon = c_f(2);
  
  % Initialise maps
  g_d     = [floor(size(m_p_in,1)/n_lat), floor(size(m_p_in,2)/n_lon)];
  m_occ   = zeros(g_d(1), g_d(2));
  m_r     = zeros(g_d(1), g_d(2));

  % Calculate occupancy map and raster map
  for i = 1:g_d(1)
    for j = 1:g_d(2)
      ii = (i-1)*n_lat+1;
      jj = (j-1)*n_lon+1;        
      % Extract data from original raster
      mat = m_p_in(ii:ii+n_lat-1,jj:jj+n_lon-1); 
      occ = sum(mat, 'all')/(n_lat*n_lon);

      % occupancy map
      m_occ(i,j) = occ;

      % raster map
      if occ > 0 
        m_r(i,j) = 1;
      end
    end
  end
end