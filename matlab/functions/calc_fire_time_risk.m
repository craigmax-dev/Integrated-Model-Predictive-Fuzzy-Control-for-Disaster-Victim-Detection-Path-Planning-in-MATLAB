% % Cell fire time risk (theory - to be tested)

function t_fire = calc_fire_time_risk(m_f, v_w)
    [n_rows, n_cols] = size(m_f);
    % Initialize with 1000 to signify cells not yet affected by fire.
    % Must be within range of input fuzzy MF
    t_fire = 100 * ones(n_rows, n_cols); 
  
    % Define relative positions for different radii
    radius1 = getRadius(1);
    radius2 = getRadius(2);
    radius3 = getRadius(3);

    for i = 1:n_rows
        for j = 1:n_cols
            % Check neighbors within the respective radii
            for rad = 1:3
                % Get the relative positions based on the radius
                switch rad
                    case 1, rel_pos = radius1;
                    case 2, rel_pos = radius2;
                    case 3, rel_pos = radius3;
                end
                
                % Extract neighbor indices within grid bounds
                neighbors = get_valid_neighbors(i, j, n_rows, n_cols, rel_pos);
                
                % Check conditions based on wind velocity
                t_fire = apply_fire_rules(neighbors, m_f, v_w, rad, t_fire, i, j);
            end
        end
    end
end

function rel_pos = getRadius(radius)
    offset = -radius:radius;
    rel_pos = [offset', zeros(numel(offset), 1); zeros(numel(offset), 1), offset'];
end

function neighbors = get_valid_neighbors(i, j, n_rows, n_cols, rel_pos)
    neighbors = [];
    for k = 1:size(rel_pos, 1)
        ni = i + rel_pos(k, 1);
        nj = j + rel_pos(k, 2);
        if ni >= 1 && ni <= n_rows && nj >= 1 && nj <= n_cols
            neighbors(end+1) = sub2ind([n_rows, n_cols], ni, nj);
        end
    end
end

function t_fire = apply_fire_rules(neighbors, m_f, v_w, radius, t_fire, i, j)
    
    % Ensure that neighbors are within the bounds of m_fire
    neighbors = neighbors(neighbors > 0 & neighbors <= numel(m_f));

    if v_w == 0  % Zero wind velocity
        if radius == 1 && any(m_f(neighbors) == 3)
            t_fire(i, j) = 0;
        elseif radius == 1 && any(m_f(neighbors) == 2)
            t_fire(i, j) = min(t_fire(i, j), 2);
        elseif radius == 2 && any(m_f(neighbors) == 3)
            t_fire(i, j) = min(t_fire(i, j), 2);
        elseif radius == 2 && any(m_f(neighbors) == 2)
            t_fire(i, j) = min(t_fire(i, j), 4);
        elseif radius == 3 && any(m_f(neighbors) == 3)
            t_fire(i, j) = min(t_fire(i, j), 4);
        elseif radius == 3 && any(m_f(neighbors) == 2)
            t_fire(i, j) = min(t_fire(i, j), 6);
        end
    elseif v_w <= 5  % Low wind velocity
        if radius <= 2 && any(m_f(neighbors) == 3)
            t_fire(i, j) = 0;
        elseif radius <= 2 && any(m_f(neighbors) == 2)
            t_fire(i, j) = min(t_fire(i, j), 2);
        end
    else  % High wind velocity
        if radius <= 3 && any(m_f(neighbors) == 3)
            t_fire(i, j) = 0;
        elseif radius == 1 && any(m_f(neighbors) == 2)
            t_fire(i, j) = min(t_fire(i, j), 2);
        end
    end
end
