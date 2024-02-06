% TODO: 
% - validate max time calculation

% Theoretical max response time for agent used to normalise FIS input
function maxResponseTime = calc_maxResponseTime(l_x_s, l_y_s, n_q, n_x_s, n_y_s, t_scan_c, v_as)

  % Max travel time calculation
  % Total length and width of the search area
  totalLength = (n_x_s-1) * l_x_s;
  totalWidth = (n_y_s-1) * l_y_s;

  % Calculate the diagonal distance of the search area
  diagonalDistance = sqrt(totalLength^2 + totalWidth^2);

  % Calculate maximum response time
  maxResponseTime = (n_q + 1)*(diagonalDistance / v_as) + (n_q + 1) * t_scan_c;

end