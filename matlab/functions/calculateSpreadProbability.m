function p = calculateSpreadProbability(t_ckl, t_i, t_b)
    % Calculation of fire spread probability based on burn time
    if t_ckl <= (t_b - t_i) / 5 + t_i
        p = 4 / (t_b - t_i) * t_ckl + (0.2 * t_b - 4.2 * t_i) / (t_b - t_i);
    elseif t_ckl <= t_b
        p = 5 / (4 * (t_b - t_i)) * (-t_ckl + t_b);
    else
        p = 0; % No spread if outside of time range
    end
end