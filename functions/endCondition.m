function [outputs] = endCondition(inputs)

    % Time based
    if condition == "time"
        if ct_prog*dt_prog <= toc
            ct_prog = ct_prog + 1;
            prog    = t/t_f;
            fprintf("Simulation progress = %4.2f %% \n", prog)
        end
        if t >= t_f
            finishFlag = true;
        end        
    elseif condition == "scan"
        % Entire map scanned
        if ct_prog*dt_prog <= toc
            ct_prog = ct_prog + 1;
            prog    = sum(m_scan, 'all')/(n_x_search*n_y_search) * 100;
            fprintf("Simulation progress = %4.2f %%", prog)
        end
        if sum(m_scan, 'all') == n_x_search*n_y_search
            finishFlag = true;
        end
    end
end
