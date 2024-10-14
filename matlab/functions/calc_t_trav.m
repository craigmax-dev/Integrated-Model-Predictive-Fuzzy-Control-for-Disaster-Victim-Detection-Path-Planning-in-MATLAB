% Calculate travel time for UAV from one cell to another

% CHANGELOG
% Fixed distance calculation

% V2 REFACTOR
function [t_travel] = calc_t_trav(loc_1, loc_2, l_x_s, l_y_s, ...
  ang_w, v_w, v_as)
  % Distance in m
  d     = sqrt(...
      ((l_x_s*(loc_2(1) - loc_1(1))).^2) + ...
      ((l_y_s*(loc_2(2) - loc_1(2))).^2));
  
  % Ground angle
  a_g   = atan2(loc_2(2) - loc_1(2), loc_2(1) - loc_1(1));

  % Wind to track angle
  a_wt  = a_g - ang_w;

  % Wind correction angle
  wind_ratio = v_w*sin(a_wt)/v_as;
  if abs(wind_ratio) > 1
      % Handle case where wind speed is too high relative to airspeed
      t_travel = Inf; % Indicate impossible travel due to high wind
      return;
  end
  a_wca = asin(wind_ratio);

  % Ground speed
  v_gs  = v_as*cos(a_wca) + v_w*cos(a_wt);
  
  % Add to travel time
  t_travel = d/v_gs;
  
  % Check for negative or zero ground speed
  if v_gs <= 0
      t_travel = Inf; % Indicate impossible or undefined travel time
  end
end