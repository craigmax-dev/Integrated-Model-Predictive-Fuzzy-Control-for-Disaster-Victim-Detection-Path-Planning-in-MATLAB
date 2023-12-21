%% Function attCalc()
% Return attraction of cell

% V2

% CHANGELOG
% removed valid_cells calculation for now
% Fixed bug with m_dw changing dimensions
% added valid_cells calculation to exclude cells in queue

% TODO
% validate function - initial test shows huge performance improvements
% refactor valid_cells calculation? must allow rescan
% when removing cell retrictions - will keep scanning same cells. Optimize FIS?
% Fix bug - repeat in queue. Should not assign cell which is already assigned

% V2.2
function m_att = calc_att(fis, m_t_response, m_prior, m_dw, m_scan, m_schedule)
    % Initialize the attraction map
    m_att = NaN(size(m_t_response));

    % Identify cells that are not scheduled
    valid_cells = ~m_schedule;

    % Check if there are any valid cells to process
    if ~any(valid_cells, 'all')
        return;  % Exit if no valid cells
    end

    % Prepare the inputs for FIS for valid cells
    fisInputs = [m_t_response(valid_cells), m_prior(valid_cells)];

    % Ensure inputs are of correct type
    fisInputs = double(fisInputs);
    
    % Vectorized FIS evaluation for valid cells
    m_att(valid_cells) = evalfis(fis, fisInputs);

end


% V2.2
% function m_att = calc_att(fis, m_t_response, m_prior, m_dw, m_scan, m_schedule)
%     
%     valid_cells = ~(m_schedule);
% 
%     % Prepare the inputs for FIS
%     % Reshape each matrix into a column vector and concatenate them
% %     fisInputs = [reshape(m_t_response, [], 1), reshape(m_prior, [], 1), reshape(m_dw, [], 1)];
%     fisInputs = [m_t_response(valid_cells), m_prior(valid_cells), m_dw(valid_cells)];
% 
%     % Ensure inputs are of correct type
%     fisInputs = double(fisInputs);
%     
%     % Vectorized FIS evaluation for all cells
%     % Reshape the output back to the original matrix size
%     m_att = reshape(evalfis(fis, fisInputs), size(m_t_response));
% end
