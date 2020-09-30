function [lab_title, lab_x, lab_y, lab_legend] = func_plot_labels(data_name)
  if data_name == " "
    lab_title = "";
    lab_x = "Latitude";
    lab_y = "Longitude";
    lab_legend = "";
  elseif data_name == "m_dw_hist_animate"
    lab_title = ;
    lab_x = "Latitude";
    lab_y = "Longitude";
    lab_legend = "";
   elseif data_name == "m_scan_hist_animate"
    lab_title = ;
    lab_x = "Latitude";
    lab_y = "Longitude";
    lab_legend = "";
  end    
end