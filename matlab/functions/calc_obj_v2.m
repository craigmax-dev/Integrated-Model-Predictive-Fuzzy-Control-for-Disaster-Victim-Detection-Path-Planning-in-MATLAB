%% Function objEval
% Evaluate objective function

% TODO
% - consider alternate configurations of objective function - could combine
% priority and certainty into single figure by multiplying, then adjust based on
% proximity to fire as an additional risk (multiply by fire prox?/downwind map)

% Comments for Anahita and Mirko
% - not confident on their formulation of objective - can't be output of FIS
% because then the MPC can minimise it by adjusting the FIS parameters. Also
% objective should depend on unknown parameters which FIS has no knowledge of.
% - Is there a way to tie in knowledge of uncerrtain parameters and
% certainty/sensor accuracy parameters? - i.e. if sensor accuracy is wrong, we
% may misidentify victims?

% V2 REFACTOR V5
function [s_obj, obj] = calc_obj_v2(weight, m_f, m_bo_s, m_scan, m_victim_s, dt_s, s_obj, c_f_s)
    
  % Active fires map
  m_fo = (m_f == 3);

  % Coarsen environment maps to agent resolution
  m_fo_s = func_coarsen(m_fo, c_f_s); 

  % Victim parameter: Use victim map if available, else use building occupancy as a proxy
  if isempty(m_victim_s)
      m_victim_s = m_bo_s;  % Proxy for victim likelihood
  end

  % priorInputs = struct;
  % 
  % for i = 1:numel(fis.Inputs)
  %   inputName = fis.Inputs(i).Name;
  % 
  % 
  %   switch inputName
  %     case 'priority_first_scan'
  %         priorInputs.priority_first_scan = double(m_scan == 0) .* m_victim_s .* weight.first_scan;
  %     case 'priority_dw' % Think about which to use
  %         priorInputs.priority_dw = m_victim_s .* m_fo_s .* weight.fire;
  %         % priorInputs.priority_dw = m_victim_s .* (1 - m_dw) .* weight.fire;
  %     case 'cell_priority' % TBC...
  %         priorInputs.cell_priority = double(m_scan == 0) .* m_victim_s .* weight.first_scan;
  %         % priorInputs.cell_priority = agent_model.m_bo_s;
  %     case 'cell_scan_certainty' % TO DO: rename weight.rescan
  %       priorInputs.cell_scan_certainty = m_victim_s .* (1 - agent_model.m_scan) .* weight.repeat_scan;
  %     case 'cell_fire_time_risk' % TBC...
  %       priorInputs.cell_fire_time_risk = m_victim_s .* m_fo_s .* weight.fire;
  %     %   priorInputs.cell_fire_time_risk = calc_fire_time_risk(agent_model.m_f_s, v_w);
  %   end
  % end

  m_prior_first_scan = double(m_scan == 0) .* m_victim_s .* weight.first_scan;
  m_prior_fire = m_victim_s .* m_fo_s .* weight.fire;
  m_prior_certainty = m_victim_s .* (1 - m_scan) .* weight.repeat_scan;

  m_prior = m_prior_first_scan + m_prior_certainty + m_prior_fire;

  % Ensure the objective function is always positive
  obj = sum(m_prior, 'all') * dt_s;
  s_obj = s_obj + obj; % Sum of objective over time
end
