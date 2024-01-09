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

% TODO
% Update idle task?

% agent_model.a_task
% 1 = travel
% 2 = scan
% 3 = idle

function agent_model = model_agent(agent_model, v_w, ang_w, dt_a, k, dt_s)
              
  for a = 1:agent_model.n_a

    % Skip task if battery level is 0 or less
    if agent_model.a_battery_level(a) < dt_a
        continue; % Skip to the next agent
    else
      agent_model.a_battery_level(a) = agent_model.a_battery_level(a) - dt_a; % Reduce battery level
    end
        
    if agent_model.a_task(a) == 1
      
      % Travel
      agent_model.a_t_trav(a) = agent_model.a_t_trav(a) - dt_a;
      
      if agent_model.a_t_trav(a) <= 0
        
        % Set task to scan
        agent_model.a_task(a)   = 2;                            
        
        % Update agent location
        agent_model.a_loc(a, :) = agent_model.a_target(a, :, 1);
        
        % Update agent location history
        agent_model.a_loc_hist    = [agent_model.a_loc_hist; agent_model.a_loc(a, :), a, k];        
        
        % Initialise remaining scantime, using additional time from the travel
        agent_model.a_t_scan(a) = agent_model.m_t_scan(agent_model.a_loc(a, 1), agent_model.a_loc(a, 2)) + agent_model.a_t_trav(a);
      
      end
      
    elseif agent_model.a_task(a) == 2
      
      % Scan
      agent_model.a_t_scan(a) = agent_model.a_t_scan(a) - dt_a;
      
      if agent_model.a_t_scan(a) <= 0
        
        % Set task to travel
        agent_model.a_task(a) = 1;
        
        % Get cell coordinates
        i = agent_model.a_loc(a, 1);
        j = agent_model.a_loc(a, 2);
        
        % Scan cell
        agent_model.m_scan(i, j) = k * dt_s;
        
        % Shift target list
        agent_model.a_target(a, 1, :)   = circshift(agent_model.a_target(a, 1, :), -1);
        agent_model.a_target(a, 2, :)   = circshift(agent_model.a_target(a, 2, :), -1);        
        agent_model.a_target(a, :, end) = [NaN, NaN];
        
        % Set task to idle if no target, otherwise calculate travel time
        if(isnan(agent_model.a_target(a, :, 1)))
          
          agent_model.a_task(a) = 3;
        
        else
          
          agent_model.a_t_trav(a) = calc_t_trav(agent_model.a_loc(a, :), agent_model.a_target(a, :, 1), ...
            agent_model.l_x_s, agent_model.l_y_s, ang_w, v_w, agent_model.v_as);
        
        end
        
      end
      
    elseif (agent_model.a_task == 3)
      
      return
    
    end

  end
  
end