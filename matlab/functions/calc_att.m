%% Function attCalc()
% Return attraction of cell

% V2

% CHANGELOG
% removed valid_cells calculation for now
% Fixed bug with m_dw changing dimensions
% added valid_cells calculation to exclude cells in queue
% Removed t_dw and m_scan from inputs

% TODO
% validate function - initial test shows huge performance improvements
% refactor valid_cells calculation? must allow rescan
% when removing cell retrictions - will keep scanning same cells. Optimize FIS?
% Fix bug - repeat in queue. Should not assign cell which is already assigned
% Add m_dw to attraction calculation? Or is this priority instead?
% Refactor: change how valid_cells is defined. If an agent runs out of battery
% we need to clear/reset parameters related to it. We also cannot consider plan
% by agents at higher level.
% Remove recalculation of n_x_s, n_y_s

% Refactor:
% Valid cells different for each agent

% TODO
% Refactor function - does not seem to work correctly. scheduled_cells is not the correct size 

% V2.4
function m_att = calc_att(fis, m_t_response, m_prior, a_target, communication_enabled)
    % Initialize the attraction map
    m_att = NaN(size(m_t_response));

    % Create a logical matrix indicating which cells are scheduled
    scheduled_cells = false(size(m_t_response)); % Initialize with all false
    if communication_enabled
        % If communication is enabled, consider all scheduled cells by all agents
        for agentIdx = 1:size(a_target, 3)
            for targetIdx = 1:size(a_target, 1)
                x = a_target(targetIdx, 1, agentIdx);
                y = a_target(targetIdx, 2, agentIdx);
                if x > 0 && y > 0
                    scheduled_cells(x, y) = true;
                end
            end
        end
    else
        % If communication is disabled, consider only the agent's own scheduled cells
        for targetIdx = 1:size(a_target, 1)
            x = a_target(targetIdx, 1);
            y = a_target(targetIdx, 2);
            if x > 0 && y > 0
                scheduled_cells(x, y) = true;
            end
        end
    end

    % Identify cells that are not scheduled and do not contain NaN in inputs
    valid_cells = ~scheduled_cells & ~isnan(m_t_response) & ~isnan(m_prior);

    % Check if there are any valid cells to process
    if ~any(valid_cells, 'all')
        fprintf("No valid cells in calc_att\n")
        return;  % Exit if no valid cells
    end

    % Prepare the inputs for FIS for valid cells
    fisInputs = [m_t_response(valid_cells), m_prior(valid_cells)];

    % Ensure inputs are of correct type
    fisInputs = double(fisInputs);
    
    % Vectorized FIS evaluation for valid cells
    m_att(valid_cells) = evalfis(fis, fisInputs);
end
