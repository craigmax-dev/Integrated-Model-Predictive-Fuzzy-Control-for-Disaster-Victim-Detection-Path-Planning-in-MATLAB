% V2
% CHANGELOG
% Refactor to remove agent indexing from function
% BUGFIX: Prevent choice of NaN cell.

% V2.1
function [a_target_updated] = func_taskAssignment(m_att)

    % % Initialize the updated target matrix to the current target matrix
    % a_target_updated = a_target_agent;

    % Check if there are valid (non-NaN) values in m_att
    if any(~isnan(m_att), 'all')
        % Find cell with maximum attraction, ignoring NaN values
        [i, j] = find(m_att == max(m_att(:)), 1);

        % Assign this cell as the target for the queue position q
        a_target_updated = [i, j];
    else
        % If all values are NaN, assign NaN to the target
        a_target_updated = NaN;
    end
end