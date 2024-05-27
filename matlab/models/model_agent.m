% Simulate agent system
% Date:     01/12/2023
% Author:   Craig Maxwell

% TODO: improvements: battery recharge locations do not have priority

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

% agent_model.a_task
% 1 = travel
% 2 = scan
% 3 = idle

% V2.4 - battery recharge
% NOTE: Battery model not properly implemented in the remaining code
function agent_model = model_agent(agent_model, v_w, ang_w, m_f, m_dw, config)

  % Decrease agent scan certainty
  agent_model.m_scan = max(agent_model.m_scan - config.sigma, 0);

  % Update downwind and fire search maps
  agent_model.m_dw_s = func_coarsen(m_dw, config.c_f_s); 
  agent_model.m_f_s = func_coarsen(m_f, config.c_f_s); 

  for a = 1:agent_model.n_a

    % Skip task if battery level is 0 or less
    if agent_model.a_battery_level(a) < config.dt_a
      continue; % Skip to the next agent
    else
      agent_model.a_battery_level(a) = agent_model.a_battery_level(a) - config.dt_a; % Reduce battery level
    end

    remaining_dt = config.dt_a; % Time remaining in this timestep

    while remaining_dt > 0
      % Check and process tasks based on the current task of the agent
      switch agent_model.a_task(a)
        case 1 % Travel
          travel_time_needed = agent_model.a_t_trav(a);
          time_spent = min(travel_time_needed, remaining_dt);
          agent_model.a_t_trav(a) = agent_model.a_t_trav(a) - time_spent;
          remaining_dt = remaining_dt - time_spent;

          if agent_model.a_t_trav(a) <= 0
            % Update agent location and history
            agent_model.a_loc(a, :) = agent_model.a_target(a, :, 1);
            
            % Check if at a recharge station
            if agent_model.m_recharge(agent_model.a_loc(a, 1), agent_model.a_loc(a, 2)) == 1
              agent_model.a_task(a) = 3; % Set task to recharge
              agent_model.a_t_recharge(a) = agent_model.t_recharge; % Set recharge time
            else
              % Change task to scan and calculate scan time
              agent_model.a_task(a) = 2;
              agent_model.a_t_scan(a) = agent_model.m_t_scan(agent_model.a_loc(a, 1), agent_model.a_loc(a, 2));
            end
          end

        case 2 % Scan
          scan_time_needed = agent_model.a_t_scan(a);
          time_spent = min(scan_time_needed, remaining_dt);
          agent_model.a_t_scan(a) = agent_model.a_t_scan(a) - time_spent;
          remaining_dt = remaining_dt - time_spent;

          if agent_model.a_t_scan(a) <= 0
            % Perform scan logic
            i = agent_model.a_loc(a, 1);
            j = agent_model.a_loc(a, 2);
            agent_model.m_scan(i, j) = max(agent_model.m_scan(i,j) - config.sigma, agent_model.sensor_accuracy);

            % Shift target list
            agent_model.a_target(a, 1, :) = circshift(agent_model.a_target(a, 1, :), -1);
            agent_model.a_target(a, 2, :) = circshift(agent_model.a_target(a, 2, :), -1);
            agent_model.a_target(a, :, end) = [NaN, NaN];

            % Check if target list is depleted
            if isnan(agent_model.a_target(a, 1, 1))
              warning('Agent %d has depleted its target list.', a);
              remaining_dt = 0;
            else
              % Calculate new travel time
              agent_model.a_t_trav(a) = calc_t_trav(agent_model.a_loc(a, :), agent_model.a_target(a, :, 1), ...
                agent_model.l_x_s, agent_model.l_y_s, ang_w, v_w, agent_model.v_as);
              agent_model.a_task(a) = 1; % Set task to travel
            end
          end

        case 3 % Recharge
          fprintf("Agent %i recharging\n", a)
          recharge_time_needed = agent_model.a_t_recharge(a);
          time_spent = min(recharge_time_needed, remaining_dt);
          agent_model.a_t_recharge(a) = agent_model.a_t_recharge(a) - time_spent;
          remaining_dt = remaining_dt - time_spent;

          if agent_model.a_t_recharge(a) <= 0
            % Recharge complete
            agent_model.a_battery_level(a) = agent_model.a_battery_level_i(a); % Fully recharge the battery
            % Shift target list
            agent_model.a_target(a, 1, :) = circshift(agent_model.a_target(a, 1, :), -1);
            agent_model.a_target(a, 2, :) = circshift(agent_model.a_target(a, 2, :), -1);
            agent_model.a_target(a, :, end) = [NaN, NaN];

            % Check if target list is depleted
            if isnan(agent_model.a_target(a, 1, 1))
              warning('Agent %d has depleted its target list.', a);
              remaining_dt = 0;
            else
              % Calculate new travel time
              agent_model.a_t_trav(a) = calc_t_trav(agent_model.a_loc(a, :), agent_model.a_target(a, :, 1), ...
                agent_model.l_x_s, agent_model.l_y_s, ang_w, v_w, agent_model.v_as);
              agent_model.a_task(a) = 1; % Set task to travel
            end
          end

        otherwise
          % Handle unknown task state, if needed
          break; % Exit the while loop
      end
    end    
  end
end


% % V2.3 - refactor with no idle
% function agent_model = model_agent(agent_model, v_w, ang_w, m_f, m_dw, config)
% 
%   % Decrease agent scan certainty
%   agent_model.m_scan = max(agent_model.m_scan - config.sigma, 0);
% 
%   % Update downwind and fire search maps
%   agent_model.m_dw_s = func_coarsen(m_dw, config.c_f_s); 
%   agent_model.m_f_s = func_coarsen(m_f, config.c_f_s); 
% 
%   for a = 1:agent_model.n_a
% 
%     % Skip task if battery level is 0 or less
%     if agent_model.a_battery_level(a) < config.dt_a
%       continue; % Skip to the next agent
%     else
%       agent_model.a_battery_level(a) = agent_model.a_battery_level(a) - config.dt_a; % Reduce battery level
%     end
% 
%     remaining_dt = config.dt_a; % Time remaining in this timestep
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
%             % agent_model.a_loc_hist = [agent_model.a_loc_hist; agent_model.a_loc(a, :), a, config.k];
%             % Change task to scan and calculate scan time
%             agent_model.a_task(a) = 2;
%             agent_model.a_t_scan(a) = agent_model.m_t_scan(agent_model.a_loc(a, 1), agent_model.a_loc(a, 2));
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
%             % agent_model.m_scan(i, j) = config.k * config.dt_s; % Old
%             % formulation for rescan
% 
%             % Scan certainty implementation
%             agent_model.m_scan(i, j) = max(agent_model.m_scan(i,j) - config.sigma, agent_model.sensor_accuracy);
% 
%             % Shift target list
%             agent_model.a_target(a, 1, :) = circshift(agent_model.a_target(a, 1, :), -1);
%             agent_model.a_target(a, 2, :) = circshift(agent_model.a_target(a, 2, :), -1);
%             agent_model.a_target(a, :, end) = [NaN, NaN];
% 
%             % Check if target list is depleted
%             if isnan(agent_model.a_target(a, 1, 1))
%               warning('Agent %d has depleted its target list.', a);
%               remaining_dt = 0;
%             else
%               % Calculate new travel time
%               agent_model.a_t_trav(a) = calc_t_trav(agent_model.a_loc(a, :), agent_model.a_target(a, :, 1), ...
%                 agent_model.l_x_s, agent_model.l_y_s, ang_w, v_w, agent_model.v_as);
%               agent_model.a_task(a) = 1; % Set task to travel
%             end
%           end
% 
%         otherwise
%           % Handle unknown task state, if needed
%           break; % Exit the while loop
%       end
%     end    
%   end
% end

