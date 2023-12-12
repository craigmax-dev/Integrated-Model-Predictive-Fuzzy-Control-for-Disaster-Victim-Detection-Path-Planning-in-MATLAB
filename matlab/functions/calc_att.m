%% Function attCalc()
% Return attraction of cell


% TODO
% validate function - initial test shows huge performance improvements.

function m_att = calc_att(fis, m_t_response, m_prior, m_dw, m_scan, m_schedule)
    % Initialize the attraction map
    m_att = NaN(size(m_t_response));

    % Identify cells that are neither scanned nor scheduled
    valid_cells = ~(m_scan | m_schedule);

    % Check if there are any valid cells to process
    if ~any(valid_cells, 'all')
        return;  % Exit if no valid cells
    end

    % Prepare the inputs for FIS
    fisInputs = [m_t_response(valid_cells), m_prior(valid_cells), m_dw(valid_cells)];

    % Ensure inputs are of correct type
    fisInputs = double(fisInputs);
    
    % Vectorized FIS evaluation for valid cells
    m_att(valid_cells) = evalfis(fis, fisInputs);
end


% % V2 vectorized
% function m_att = calc_att(fis, m_t_response, m_prior, m_dw, m_scan, m_schedule)
%     % Initialize the attraction map
%     m_att = NaN(size(m_t_response));
% 
%     % Identify cells that are neither scanned nor scheduled
%     valid_cells = ~(m_scan | m_schedule);
% 
%     % Vectorized FIS evaluation for valid cells
%     m_att(valid_cells) = evalfis(fis, [m_t_response(valid_cells), m_prior(valid_cells), m_dw(valid_cells)]);
% 
%     % For cells that are scanned or scheduled, the attraction is NaN
%     % This is already handled by the initialization of m_att
% end


% V1
% function [att] = calc_att(fis, t_response, p_prior, t_dw, scan_state, schedule_state)
%   if scan_state || schedule_state
%    att = NaN;
%   else
%     att = evalfis(fis, [t_response, p_prior, t_dw]);
%   end
% end
