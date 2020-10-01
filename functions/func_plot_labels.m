% Function func_plot_labels()
% Return labels for plots  

%% TO ADD:
% - FIS inputs
% - FIS outputs

function [lab_title, lab_x, lab_y, lab_legend] = func_plot_labels(data_name)
  if data_name == "m_f_hist_animate"
    lab_title = "";
    lab_x = "Latitude";
    lab_y = "Longitude";
    lab_legend = "";
  elseif data_name == "m_dw_hist_animate"
    lab_title = "";
    lab_x = "Latitude";
    lab_y = "Longitude";
    lab_legend = "";
  elseif data_name == "m_bo"
    lab_title = "";
    lab_x = "Latitude";
    lab_y = "Longitude";
    lab_legend = "Building occupancy";
  elseif data_name == "m_prior"
    lab_title = "";
    lab_x = "Latitude";
    lab_y = "Longitude";
    lab_legend = "Cell priority";
  elseif data_name == "m_f_hist"
    lab_title = "";
    lab_x = "Latitude";
    lab_y = "Longitude";
    lab_legend = "Ignition time (s)";
  elseif data_name == "m_scan_hist"
    lab_title = "Scan time (s)";
    lab_x = "Latitude";
    lab_y = "Longitude";
    lab_legend = "Scan time (s)";
  elseif data_name == "obj_hist"
    lab_title = "Objective function over simulation";
    lab_x = "Time (s)";
    lab_y = "J";
    lab_legend = "";
  elseif data_name == "s_obj_hist"
    lab_title = "Objective function sum over simulation";
    lab_x = "Time (s)";
    lab_y = "$$ \sum_{k=0}^{t/dt_s} J $$";
    lab_legend = "";
  else
    lab_title = "";
    lab_x = "";
    lab_y = "";
    lab_legend = "";
  end
end
