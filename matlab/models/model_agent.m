% Simulate agent system
% Date:     01/12/2023
% Author:   Craig Maxwell

% V2

% CHANGELOG
% flag_scan_task updated to allow either single scan or repeat
% flag_scan_task replaced with config structure for all objective func config
% agent_model.m_scan replaced with time last scanned
% repeat scan logic moved to objective function
% removed agent_model.m_scan_hist and flag_mpc
% Refactored function to use agent_model structure
% Added agent battery model and battery level check
% Refactor: use switch for agent tasks
% Updated remaining travel time calculation to take from timestep
% Refactor: more robust method to execute agent tasks sequentially

% TODO
% Plan how to use state 3. Could be for dead battery to save FIS calculations.

% agent_model.a_task
% 1 = travel
% 2 = scan
% 3 = idle


% V2.3
function agent_model = model_agent(agent_model, v_w, ang_w, dt_a, k, dt_s)
    for a = 1:agent_model.n_a
        if agent_model.a_battery_level(a) < dt_a
            agent_model.a_task(a) = 3; % Set task to idle
            continue;
        else
            agent_model = reduce_battery(agent_model, a, dt_a);
        end

        remaining_dt = dt_a;

        while remaining_dt > 0
            switch agent_model.a_task(a)
                case 1 % Travel
                    [agent_model, remaining_dt] = handle_travel(agent_model, a, remaining_dt, k);
                case 2 % Scan
                    [agent_model, remaining_dt] = handle_scan(agent_model, a, remaining_dt, k, dt_s, ang_w, v_w);
                case 3 % Idle
                    break; % Exit the while loop
                otherwise
                    break; % Handle unknown task state
            end
        end
    end
end

function agent_model = shift_and_check_targets(agent_model, a, k, dt_s, ang_w, v_w)
    agent_model.a_target(a, :, :) = circshift(agent_model.a_target(a, :, :), -1, 3);
    agent_model.a_target(a, :, end) = [NaN, NaN];

    if isnan(agent_model.a_target(a, 1, 1))
        warning('Agent %d has depleted its target list.', a);
        agent_model.a_task(a) = 3; % Set task to idle
    else
        agent_model.a_task(a) = 1; % Set task back to travel
        % Calculate new travel time
        agent_model.a_t_trav(a) = calc_t_trav(agent_model.a_loc(a, :), agent_model.a_target(a, :, 1), ...
          agent_model.l_x_s, agent_model.l_y_s, ang_w, v_w, agent_model.v_as);
    end
end

function [agent_model, remaining_dt] = handle_scan(agent_model, a, remaining_dt, k, dt_s, ang_w, v_w)
    scan_time_needed = agent_model.a_t_scan(a);
    time_spent = min(scan_time_needed, remaining_dt);
    agent_model.a_t_scan(a) = agent_model.a_t_scan(a) - time_spent;

    if agent_model.a_t_scan(a) <= 0
        i = agent_model.a_loc(a, 1);
        j = agent_model.a_loc(a, 2);
        agent_model.m_scan(i, j) = k * dt_s;
        agent_model = shift_and_check_targets(agent_model, a, k, dt_s, ang_w, v_w);
    end

    % Update remaining_dt and return it with agent_model
    remaining_dt = remaining_dt - time_spent;
end

function [agent_model, remaining_dt] = handle_travel(agent_model, a, remaining_dt, k)

    if isnan(agent_model.a_target(a, 1, 1))
        agent_model.a_task(a) = 3; % Set task to idle if the next target is NaN
        return;
    end

    travel_time_needed = agent_model.a_t_trav(a);
    time_spent = min(travel_time_needed, remaining_dt);
    agent_model.a_t_trav(a) = agent_model.a_t_trav(a) - time_spent;

    if agent_model.a_t_trav(a) <= 0
        agent_model.a_loc(a, :) = agent_model.a_target(a, :, 1);
        agent_model.a_loc_hist = [agent_model.a_loc_hist; agent_model.a_loc(a, :), a, k];
        agent_model.a_task(a) = 2; % Change task to scan
        agent_model.a_t_scan(a) = agent_model.m_t_scan(agent_model.a_loc(a, 1), agent_model.a_loc(a, 2));
    end

    % Update remaining_dt and return it with agent_model
    remaining_dt = remaining_dt - time_spent;
    
end

function agent_model = reduce_battery(agent_model, a, dt_a)
    agent_model.a_battery_level(a) = agent_model.a_battery_level(a) - dt_a;
end

% % V2.31
% function agent_model = model_agent(agent_model, v_w, ang_w, dt_a, k, dt_s)
%     for a = 1:agent_model.n_a
%         if agent_model.a_battery_level(a) < dt_a
%             agent_model.a_task(a) = 3; % Set task to idle
%             continue;
%         else
%             agent_model = reduce_battery(agent_model, a, dt_a);
%         end
% 
%         remaining_dt = dt_a;
% 
%         while remaining_dt > 0
%             switch agent_model.a_task(a)
%                 case 1 % Travel
%                     [agent_model, remaining_dt] = handle_travel(agent_model, a, remaining_dt, k);
%                 case 2 % Scan
%                     [agent_model, remaining_dt] = handle_scan(agent_model, a, remaining_dt, k, dt_s);
%                 case 3 % Idle
%                     break; % Exit the while loop
%                 otherwise
%                     break; % Handle unknown task state
%             end
%         end
%     end
% end
% 
% function agent_model = shift_and_check_targets(agent_model, a, k, dt_s)
%     agent_model.a_target(a, :, :) = circshift(agent_model.a_target(a, :, :), -1, 3);
%     agent_model.a_target(a, :, end) = [NaN, NaN];
% 
%     if isnan(agent_model.a_target(a, 1, 1))
%         warning('Agent %d has depleted its target list.', a);
%         agent_model.a_task(a) = 3; % Set task to idle
%     else
%         agent_model.a_task(a) = 1; % Set task back to travel
%         % Calculate new travel time
%         agent_model.a_t_trav(a) = calc_t_trav(agent_model.a_loc(a, :), agent_model.a_target(a, :, 1), ...
%           agent_model.l_x_s, agent_model.l_y_s, ang_w, v_w, agent_model.v_as);
%     end
% end
% 
% function [agent_model, remaining_dt] = handle_scan(agent_model, a, remaining_dt, k, dt_s)
%     scan_time_needed = agent_model.a_t_scan(a);
%     time_spent = min(scan_time_needed, remaining_dt);
%     agent_model.a_t_scan(a) = agent_model.a_t_scan(a) - time_spent;
% 
%     if agent_model.a_t_scan(a) <= 0
%         i = agent_model.a_loc(a, 1);
%         j = agent_model.a_loc(a, 2);
%         agent_model.m_scan(i, j) = k * dt_s;
%         agent_model = shift_and_check_targets(agent_model, a, k, dt_s);
%     end
% 
%     % Update remaining_dt and return it with agent_model
%     remaining_dt = remaining_dt - time_spent;
% end
% 
% function [agent_model, remaining_dt] = handle_travel(agent_model, a, remaining_dt, k)
% 
%     if isnan(agent_model.a_target(a, 1, 1))
%         agent_model.a_task(a) = 3; % Set task to idle if the next target is NaN
%         return;
%     end
% 
%     travel_time_needed = agent_model.a_t_trav(a);
%     time_spent = min(travel_time_needed, remaining_dt);
%     agent_model.a_t_trav(a) = agent_model.a_t_trav(a) - time_spent;
% 
%     if agent_model.a_t_trav(a) <= 0
%         agent_model.a_loc(a, :) = agent_model.a_target(a, :, 1);
%         agent_model.a_loc_hist = [agent_model.a_loc_hist; agent_model.a_loc(a, :), a, k];
%         agent_model.a_task(a) = 2; % Change task to scan
%         agent_model.a_t_scan(a) = agent_model.m_t_scan(agent_model.a_loc(a, 1), agent_model.a_loc(a, 2));
%     end
% 
%     % Update remaining_dt and return it with agent_model
%     remaining_dt = remaining_dt - time_spent;
% 
% end
% 
% function agent_model = reduce_battery(agent_model, a, dt_a)
%     agent_model.a_battery_level(a) = agent_model.a_battery_level(a) - dt_a;
% end


% % V2.2
% function agent_model = model_agent(agent_model, v_w, ang_w, dt_a, k, dt_s)
% 
%   for a = 1:agent_model.n_a
% 
%     % Skip task if battery level is 0 or less
%     if agent_model.a_battery_level(a) < dt_a
%       agent_model.a_task(a) = 3; % Set task to idle
%       continue; % Skip to the next agent
%     else
%       agent_model.a_battery_level(a) = agent_model.a_battery_level(a) - dt_a; % Reduce battery level
%     end
% 
%     remaining_dt = dt_a; % Time remaining in this timestep
% 
%     while remaining_dt > 0
%       % Check and process tasks based on the current task of the agent
%       switch agent_model.a_task(a)
%         case 1 % Travel
%           travel_time_needed = agent_model.a_t_trav(a);
%           time_spent = min(travel_time_needed, remaining_dt);
%           agent_model.a_t_trav(a) = agent_model.a_t_trav(a) - time_spent;
%           remaining_dt = remaining_dt - time_spent;
% 
%           if agent_model.a_t_trav(a) <= 0
%             % Update agent location and history
%             agent_model.a_loc(a, :) = agent_model.a_target(a, :, 1);
%             agent_model.a_loc_hist = [agent_model.a_loc_hist; agent_model.a_loc(a, :), a, k];
%             % Change task to scan and calculate scan time
%             agent_model.a_task(a) = 2;
%             agent_model.a_t_scan(a) = agent_model.m_t_scan(agent_model.a_loc(a, 1), agent_model.a_loc(a, 2));
% 
%           end
% 
%         case 2 % Scan
%           scan_time_needed = agent_model.a_t_scan(a);
%           time_spent = min(scan_time_needed, remaining_dt);
%           agent_model.a_t_scan(a) = agent_model.a_t_scan(a) - time_spent;
%           remaining_dt = remaining_dt - time_spent;
% 
%           if agent_model.a_t_scan(a) <= 0
%             % Perform scan logic
%             i = agent_model.a_loc(a, 1);
%             j = agent_model.a_loc(a, 2);
%             agent_model.m_scan(i, j) = k * dt_s;
% 
%             % Shift target list
%             agent_model.a_target(a, 1, :) = circshift(agent_model.a_target(a, 1, :), -1);
%             agent_model.a_target(a, 2, :) = circshift(agent_model.a_target(a, 2, :), -1);
%             agent_model.a_target(a, :, end) = [NaN, NaN];
% 
%             % Check if target list is depleted
%             if isnan(agent_model.a_target(a, 1, 1))
%               warning('Agent %d has depleted its target list.', a);
%               agent_model.a_task(a) = 1; % Set task to idle
%               remaining_dt = 0;
%             else
%               % Calculate new travel time
%               agent_model.a_t_trav(a) = calc_t_trav(agent_model.a_loc(a, :), agent_model.a_target(a, :, 1), ...
%                 agent_model.l_x_s, agent_model.l_y_s, ang_w, v_w, agent_model.v_as);
%               agent_model.a_task(a) = 1; % Set task to travel
%             end
%           end
% 
%         case 3 % Idle
%           % Idle logic can be added here if needed
%           break; % Exit the while loop
% 
%         otherwise
%           % Handle unknown task state, if needed
%           break; % Exit the while loop
%       end
%     end    
%   end
% end