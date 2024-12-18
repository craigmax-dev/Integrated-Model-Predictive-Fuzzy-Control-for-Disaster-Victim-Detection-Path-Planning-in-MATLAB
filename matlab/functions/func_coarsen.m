%% Coarsen raster grid
% Generate coarsened occupancy map and raster map based on input map.
% Inputs:
% - m_p_in - input map
% - c_f - coarsening factor
% Outputs:
% m_occ - occupancy map (percent of each cell occupied)
% m_r   - raster map (binary, identifies if any occupancy in cell)
% Assumptions:
% - some error introduced by rounding - ignoring rows and column on sides
% of matrix.

%% Coarsen raster
% REFACTOR - V2.2 - VECTORISATION
function [m_occ, m_r] = func_coarsen(m_p_in, c_f)
    % Extract coarsening factors for latitude and longitude
    n_lat = c_f(1);
    n_lon = c_f(2);

    % Determine the size of the coarsened matrix
    g_d = ceil(size(m_p_in) ./ c_f);  % Using ceil to include edge cells
    m_occ = zeros(g_d(1), g_d(2));
    m_r = zeros(g_d(1), g_d(2));

    % Perform coarsening operation
    for i = 1:g_d(1)
        for j = 1:g_d(2)
            % Define block boundaries
            ii_start = (i-1)*n_lat + 1;
            ii_end = min(i*n_lat, size(m_p_in, 1));
            jj_start = (j-1)*n_lon + 1;
            jj_end = min(j*n_lon, size(m_p_in, 2));

            % Extract block data
            block = m_p_in(ii_start:ii_end, jj_start:jj_end);

            % Calculate occupancy
            occ = sum(block, 'all') / numel(block);

            % Populate occupancy and raster maps
            m_occ(i, j) = occ;
            m_r(i, j) = occ > 0;
        end
    end
end