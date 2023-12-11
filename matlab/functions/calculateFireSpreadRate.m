% Function created for environment model refactor

function p = calculateFireSpreadRate(bt, t_i, t_b)
    if bt <= (t_b - t_i) / 5 + t_i
        p = 4 / (t_b - t_i) * bt + (0.2 * t_b - 4.2 * t_i) / (t_b - t_i);
    elseif bt <= t_b
        p = 5 / (4 * (t_b - t_i)) * (-bt + t_b);
    else
        p = 0;  % No spread after burnout
    end
end