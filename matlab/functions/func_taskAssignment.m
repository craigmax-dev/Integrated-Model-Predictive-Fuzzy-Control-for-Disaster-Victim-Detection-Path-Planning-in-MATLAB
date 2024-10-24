% V2
% CHANGELOG
% Refactor to remove agent indexing from function
% BUGFIX: Prevent choice of NaN cell.
% Feature: Added battery model logic

% V2.7 BATTERY MODEL REFACTOR
function target_for_q = func_taskAssignment(q, a_target, m_att, m_recharge, a_loc, a_battery_level, a_battery_level_i, l_x_s, l_y_s, ang_w, v_w, v_as, communication_enabled)
    
    % If communication is enabled, create a logical array indicating if the cell is targeted at position q by any agent
    if communication_enabled
        other_agents_targets = reshape(a_target(:, :, q), [], 2); % Reshape for logical indexing
        already_targeted = ismember([1:size(m_att, 1)]', other_agents_targets(:, 1)) & ...
                           ismember([1:size(m_att, 2)]', other_agents_targets(:, 2));
        % Exclude already targeted cells from consideration
        m_att(already_targeted) = NaN;
    end

    % Identify the recharge stations and calculate travel times
    [recharge_i, recharge_j] = find(m_recharge == 1);
    recharge_locations = [recharge_i, recharge_j];
    current_location = a_loc(q, :);
    
    % Calculate travel times to each recharge station
    travel_times = arrayfun(@(x) calc_t_trav(current_location, recharge_locations(x, :), l_x_s, l_y_s, ang_w, v_w, v_as), 1:size(recharge_locations, 1));
    
    % Find the nearest recharge station within the battery constraint
    battery_level = a_battery_level(q);
    max_response_time = min(travel_times(battery_level >= travel_times));
    
    if ~isempty(max_response_time)
        nearest_recharge_idx = find(travel_times == max_response_time, 1);
        nearest_recharge_location = recharge_locations(nearest_recharge_idx, :);
        
        % If nearest recharge station is found, set it as the target
        target_for_q = nearest_recharge_location;
    else
        % Check if there are valid (non-NaN) values in m_att after excluding already targeted cells
        if any(~isnan(m_att), 'all')
            % Find cell with maximum attraction, ignoring NaN values
            [max_val, ind] = max(m_att(:));
            [i, j] = ind2sub(size(m_att), ind);

            % Return this cell as the target for the queue position q
            target_for_q = [i, j]; % Return a 1-by-2 vector representing the target
        else
            % If all valid cells are already scheduled or no cells have attraction, return NaN
            target_for_q = NaN(1, 2); % Return a 1-by-2 vector of NaNs
        end
    end
end