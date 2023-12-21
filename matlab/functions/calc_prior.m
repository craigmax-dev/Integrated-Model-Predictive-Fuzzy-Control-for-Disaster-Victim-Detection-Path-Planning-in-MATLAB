% CHANGELOG
% Refactored for objective function refactor: added first scan and rescan
% component, optional victim map, 

% V2 - refactor for cell re-scan
function m_prior = calc_prior(m_bo_s, m_dw_s, m_scan_s, t, config, m_victim_s)
    % Check if m_victim_s is provided, else use m_bo_s
    if nargin < 7
        m_victim_s = m_bo_s;
    end

    % First-time scan priority
    % Using building occupancy as a basis for unscanned cells
    m_P_first_scan = double(m_scan_s == 0) .* m_bo_s * config.weight_first_scan;

    % Re-scan priority
    % Using victim map (if scanned) or building occupancy (if not scanned)
    time_since_scan = max(t - m_scan_s, 0);  % Ensuring no negative values
    
    m_P_re_scan = double(m_scan_s ~= 0) .* m_victim_s .* ...
              (config.weight_repeat_scan ./ (1 + time_since_scan));

    % Incorporate downwind map and travel time
    % Assuming proximity to fire (lower m_dw_s) increases priority, and longer travel time decreases priority
    m_P_dw = (1 - m_dw_s) * config.weight_dw;  % Higher priority for cells closer to fire

    % Calculate overall priority
    m_prior = m_P_first_scan + m_P_re_scan + m_P_dw;
end