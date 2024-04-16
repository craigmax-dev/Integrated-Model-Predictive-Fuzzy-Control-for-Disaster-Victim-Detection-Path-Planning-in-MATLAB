function minQueueLength = calculateMinimumQueueLength(t_scan_c, l_x_s, v_as, v_w, config)
    % Assumption: cell lengths are the same in x and y axis

    % Calculate the total time required for scanning and traveling to one cell
    minTaskTime = t_scan_c + (l_x_s/(v_as + v_w));
    
    % Calculate the minimum queue length by dividing the MPC step time by the total task time
    % Use ceil to ensure that the queue length is an integer that fully accommodates the fractional part
    minQueueLength = ceil(config.dk_pred*config.dt_s / minTaskTime);
end