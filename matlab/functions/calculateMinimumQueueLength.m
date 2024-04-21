function l_queue_max = calculateMinimumQueueLength(t_scan_c, l_x_s, v_as, v_w, config)
  % Assumption: cell lengths are the same in x and y axis
  t_trav_min = l_x_s / (v_as + v_w);

  % Calculate the total time required for scanning and traveling to one cell
  t_cell_min = t_scan_c + t_trav_min;
  
  % Calculate the max queue length by dividing prediction horizon by the minimum time per cell
  l_queue_max = ceil(config.dk_pred*config.dt_s / t_cell_min);
end