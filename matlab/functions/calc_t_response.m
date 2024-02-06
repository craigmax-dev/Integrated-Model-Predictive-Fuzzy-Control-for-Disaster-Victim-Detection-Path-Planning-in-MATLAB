
% CHANGELOG
% Normalization using maxTravelTime instead of normalization at each
% time step
% Bugfix: Returns NAN when agent task list depleted.

% TODO
% Refactor: move agent iteration outside function
% BUGFIX: treating NaN values and breaks
% BUGFIX: 

% V2.1 
% V2 refactor: performance improvements and handling depleted task list
function m_t_response = calc_t_response(...
          n_x_s, n_y_s, l_x_s, l_y_s, ...
          n_a, a_t_scan, a_t_trav, a_target, q, ...
          ang_w, v_w, v_as, m_t_scan, maxTravelTime)

    % Initialize the response time matrix
    m_t_response = NaN(n_x_s, n_y_s, n_a);

    % Create matrices for all end locations (i, j)
    [end_locs_x, end_locs_y] = meshgrid(1:n_x_s, 1:n_y_s);
    end_locs = [end_locs_x(:), end_locs_y(:)];

    for a = 1:n_a
        % Initialize travel time for current task
        t_response_base = a_t_trav(a) + a_t_scan(a);

        % Calculate travel time for tasks in queue
        for idx = 1:size(end_locs, 1)
            loc_2 = end_locs(idx, :);
            t_response = t_response_base;

            for p = 1:q-1
                loc_1 = a_target(a, :, p);

                % Check if task list is depleted (loc_1 contains NaN)
                if any(isnan(loc_1))
                    break;  % Exit the inner loop if task list is depleted
                end

                if p ~= q-1
                    loc_next = a_target(a, :, p+1);
                else
                    loc_next = loc_2;
                end

                % Calculate travel time
                t_travel = calc_t_trav(loc_1, loc_next, l_x_s, l_y_s, ang_w, v_w, v_as);
                t_response = t_response + t_travel + m_t_scan(loc_2(1), loc_2(2));
            end

            % Assign t_response only if the task list is not depleted
            if ~any(isnan(loc_1))
                m_t_response(end_locs_x(idx), end_locs_y(idx), a) = t_response;
            end
        end
    end

    % Normalize time input in range [0, 1]
    % Replace NaNs with a default value or handle them appropriately
    % For example, you might use maxTravelTime for agents with depleted task lists
    m_t_response(isnan(m_t_response)) = maxTravelTime;
    m_t_response = m_t_response ./ maxTravelTime;
end


% % V2 refactor: performance improvements
% function m_t_response = calc_t_response(...
%           n_x_s, n_y_s, l_x_s, l_y_s, ...
%           n_a, a_t_scan, a_t_trav, a_target, q, ...
%           ang_w, v_w, v_as, m_t_scan, maxTravelTime)
% 
%     % Initialize the response time matrix
%     m_t_response = NaN(n_x_s, n_y_s, n_a);
% 
%     % Create matrices for all end locations (i, j)
%     [end_locs_x, end_locs_y] = meshgrid(1:n_x_s, 1:n_y_s);
%     end_locs = [end_locs_x(:), end_locs_y(:)];
% 
%     for a = 1:n_a
%         % Initialize travel time for current task
%         t_response_base = a_t_trav(a) + a_t_scan(a);
% 
%         % Calculate travel time for tasks in queue
%         for idx = 1:size(end_locs, 1)
%             loc_2 = end_locs(idx, :);
%             t_response = t_response_base;
% 
%             for p = 1:q-1
%                 loc_1 = a_target(a, :, p);
%                 if p ~= q-1
%                     loc_next = a_target(a, :, p+1);
%                 else
%                     loc_next = loc_2;
%                 end
%                 t_travel = calc_t_trav(loc_1, loc_next, l_x_s, l_y_s, ang_w, v_w, v_as);
%                 t_response = t_response + t_travel + m_t_scan(loc_2(1), loc_2(2));
%             end
% 
%             m_t_response(end_locs_x(idx), end_locs_y(idx), a) = t_response;
%         end
%     end
% 
%     % Normalize time input in range [0, 1]
%     m_t_response = m_t_response ./ maxTravelTime;
% end
