% Function func_plot_colormaps
% Return colourmap for plot

function [cmap, cmap_axis] = func_plot_colormaps(data_name)
  if data_name == "m_f_hist_animate"
    cmap  = [ 1,   1,   1;    % 0
              0.5, 0.5, 0.5;  % 1
              1,   0.5, 0;    % 2
              1,   0,   0;    % 3
              0,   0,   0];   % 4  
    cmap_axis = [0 4];
  end
end