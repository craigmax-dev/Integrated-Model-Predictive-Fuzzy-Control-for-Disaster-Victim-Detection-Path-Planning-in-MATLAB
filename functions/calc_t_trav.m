% Calculate travel time for UAV from one cell to another

function [t_travel] = calc_t_trav(loc_1, loc_2, l_x_s, l_y_s, ...
  ang_w, v_w, v_as)
  % Distance in m
  d     = (sqrt (...
      (l_x_s*(loc_2(1) - loc_1(1)).^2) ...
  +   (l_y_s*(loc_2(2) - loc_1(2)).^2)));
  % Ground angle
  a_g   = atan2(loc_1(2) - loc_2(2), loc_1(1) - loc_2(1));
  % Wind to track angle
  a_wt  = a_g - ang_w;
  % Wind correction angle
  a_wca = asin(v_w*sin(a_wt)/v_as);
  % Ground speed
  v_gs  = v_as*cos(a_wca) + v_w*cos(a_wt);      
  % Add to travel time
  t_travel = d/v_gs;    
end