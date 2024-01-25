% V2
% CHANGELOG
% Refactor to remove agent indexing from function
% BUGFIX: Prevent choice of NaN cell.

% V2.6
function target_for_q = func_taskAssignment(q, a_target, m_att, communication_enabled)
    
    % If communication is enabled, create a logical array indicating if the cell is targeted at position q by any agent
    if communication_enabled
        other_agents_targets = reshape(a_target(:, :, q), [], 2); % Reshape for logical indexing
        already_targeted = ismember([1:size(m_att, 1)]', other_agents_targets(:, 1)) & ...
                           ismember([1:size(m_att, 2)]', other_agents_targets(:, 2));
        % Exclude already targeted cells from consideration
        m_att(already_targeted) = NaN;
    end

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







% % V2.1
% function [a_target_updated] = func_taskAssignment(m_att)
% 
%     % % Initialize the updated target matrix to the current target matrix
%     % a_target_updated = a_target_agent;
% 
%     % Check if there are valid (non-NaN) values in m_att
%     if any(~isnan(m_att), 'all')
%         % Find cell with maximum attraction, ignoring NaN values
%         [i, j] = find(m_att == max(m_att(:)), 1);
% 
%         % Assign this cell as the target for the queue position q
%         a_target_updated = [i, j];
%     else
%         % If all values are NaN, assign NaN to the target
%         a_target_updated = NaN;
%     end
% end