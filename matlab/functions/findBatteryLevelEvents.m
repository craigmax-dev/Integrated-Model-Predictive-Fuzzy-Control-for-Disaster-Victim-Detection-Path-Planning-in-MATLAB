function events = findBatteryLevelEvents(battery_levels, threshold)
    events = []; % Initialize empty array
    for i = 1:length(battery_levels)
        if battery_levels(i) < threshold
            % Assuming time_vector is aligned with battery_levels
            events(end+1).time = time_vector(i); % The time when battery level falls below the threshold
            events(end).label = 'Battery Low'; % Label for the event
        end
    end
end
