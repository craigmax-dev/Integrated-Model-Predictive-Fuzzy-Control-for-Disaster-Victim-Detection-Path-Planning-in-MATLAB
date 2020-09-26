function [finishFlag] = simEndCondition(endCondition, t, t_f, m_scan, n_x_search, n_y_search)
  % Time based
  if endCondition == "time"
    if t >= t_f
      finishFlag = true;
    else
      finishFlag = false;
    end
  % Entire map scanned;
  elseif endCondition == "scan"
    if sum(sum(m_scan)) == n_x_search*n_y_search
      finishFlag = true;
    else
      finishFlag = false;
    end
  end
end