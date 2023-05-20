function [] = report_progress(endCondition, t, t_f, m_scan, n_x_search, n_y_search)
    % Percent of time passed
    if endCondition == "time"
      prog    = t/t_f * 100;
    % Percent of map scanned
    elseif endCondition == "scan"
      prog    = sum(sum(m_scan))/(n_x_search*n_y_search) * 100;
    end
    fprintf("Simulation progress = %.2f %% \n", prog)    
end