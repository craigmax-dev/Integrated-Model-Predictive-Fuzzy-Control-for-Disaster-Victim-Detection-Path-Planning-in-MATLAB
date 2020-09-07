% Calculate travel time for UAV from one cell to another

%% Change log
% 08/08/2020 - bugfix - incorrect brackets causing incorrect distance
% calculation

function [t_travel] = travelTime(loc_1, loc_2, l_c_s_x, l_c_s_y, ...
  ang_w, v_w, v_as_UAV)
  % Distance in m
  d     = (sqrt (...
      (l_c_s_x*(loc_2(1) - loc_1(1)).^2) ...
  +   (l_c_s_y*(loc_2(2) - loc_1(2)).^2)));
  % Ground angle
  a_g   = atan2(loc_1(2) - loc_2(2), loc_1(1) - loc_2(1));
  % Wind to track angle
  a_wt  = a_g - ang_w;
  % Wind correction angle
  a_wca = asin(v_w*sin(a_wt)/v_as_UAV);
  % Ground speed
  v_gs  = v_as_UAV*cos(a_wca) + v_w*cos(a_wt);      
  % Add to travel time
  t_travel = d/v_gs;
end
