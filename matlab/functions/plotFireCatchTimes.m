function plotFireCatchTimes(m_f_series)
    % Get the dimensions of the input matrix
    [n_x_e, n_y_e, n_timesteps] = size(m_f_series);
    
    % Initialize the matrix to store the fire catch times
    fireCatchTimes = NaN(n_x_e, n_y_e);
    
    % Loop over each cell and find the first timestep where the state is 2
    for i = 1:n_x_e
        for j = 1:n_y_e
            % Find the first timestep where the state is 2
            catchFireIndex = find(m_f_series(i, j, :) == 2, 1);
            if ~isempty(catchFireIndex)
                fireCatchTimes(i, j) = catchFireIndex;
            end
        end
    end
    
    % Plot the fire catch times matrix
    figure;
    imagesc(fireCatchTimes);
    colorbar;
    colormap(parula); % Use 'parula' colormap
    % You can also use 'viridis' or 'cividis' from cmocean if you have it installed
    % colormap(cmocean('viridis'));
    title('Order in which cells catch fire');
    xlabel('Y-axis (Cells)');
    ylabel('X-axis (Cells)');
end
