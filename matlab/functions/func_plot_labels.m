% Function func_plot_labels()
% Return labels for plots  

function [lab_title, lab_x, lab_y, lab_legend, lab_cmap, pos, ylimits] = func_plot_labels(data_name, data_type)
  pos_bottomLeft = [0.25, 0.25, 0.1, 0.1];
  pos_midLeft = [0.25, 0.5, 0.1, 0.1];
  pos_topLeft = [0.25, 0.75, 0.1, 0.1];
  pos_topRight = [0.75, 0.75, 0.1, 0.1];
  
  % Defaults
  lab_cmap = ""; 
  lab_title = "";
  lab_x = "";
  lab_y = "";
  lab_legend = "";
  pos = pos_topLeft;
  ylimits = [-inf inf];

  if data_name == "m_f_hist_animate"
    lab_x = "Latitude";
    lab_y = "Longitude";
  elseif data_name == "m_dw_hist_animate"
    lab_x = "Latitude";
    lab_y = "Longitude";
  elseif data_name == "m_bo"
    lab_x = "Latitude";
    lab_y = "Longitude";
    lab_legend = "Building occupancy";
    lab_cmap = "$$p_{bo}$$";
  elseif data_name == "m_prior"
    lab_x = "Latitude";
    lab_y = "Longitude";
    lab_legend = "Cell priority";
    lab_cmap = "$$p_{prior}$$";
  elseif data_name == "m_f_hist"
    lab_x = "Latitude";
    lab_y = "Longitude";
    lab_legend = "Ignition time (s)";
    lab_cmap = "$$t_{ig}$$";
  elseif data_name == "m_scan_hist"
    lab_title = "Scan time (s)";
    lab_x = "Latitude";
    lab_y = "Longitude";
    lab_legend = "Scan time (s)";
    lab_cmap = "$$t_{scan}$$";
  elseif data_name == "obj_hist"
    lab_title = "Objective function over simulation";
    lab_x = "$t(s)$";
    if data_type == "variable"
      pos = pos_topRight;
      lab_y = "$J$";
    elseif data_type == "relative"
      pos = pos_topLeft ;      
      lab_y = "$\frac{J}{J_{1}}$";
      ylimits = [-inf 1.2];
    end
  elseif data_name == "s_obj_hist"
    if data_type == "variable"
      lab_y = '$$\sum_{k=0}^{k_{f}} J $$';
      pos = pos_topLeft;
    elseif data_type == "relative"
      lab_y = '$$\frac{\sum_{k=0}^{k_{f}}J_{n}}{\sum_{k=0}^{k_{f}}J_{1}}$$';
      pos = pos_bottomLeft;
    end
    lab_title = "Objective function sum over simulation";
    lab_x = '$t(s)$';
  end
end
