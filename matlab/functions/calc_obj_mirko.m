function [s_obj, obj] = calc_obj_mirko(agent_model, fisArray, v_w, ang_w, config)

  for q = 2:agent_model.n_q

    % Calculate normalized response time for all agents
    m_t_response = calc_t_response(...
        agent_model.n_x_s, agent_model.n_y_s, agent_model.l_x_s, agent_model.l_y_s, ...
        agent_model.n_a, agent_model.a_t_scan, agent_model.a_t_trav, agent_model.a_target, 2, ...
        ang_w, v_w, agent_model.v_as, agent_model.m_t_scan, agent_model.maxResponseTime);

    for a = 1:agent_model.n_a
        fis = fisArray(a);
        fisInputs = struct;
        fisInputs.t_response = m_t_response(:,:,a);
        fisInputs.cell_priority = agent_model.m_bo_s;
        fisInputs.cell_scan_certainty = agent_model.m_scan;
        fisInputs.cell_fire_time_risk = calc_fire_time_risk(agent_model.m_f_s, v_w);

        % Generate attraction map using prepared FIS inputs
        m_att(:, :, a) = calc_att(fis, fisInputs, agent_model.a_target, config.flag_communication_model);
    end
  end

  % Negative the sum of attraction to turn it into a maximisation
  obj = -sum(m_att(~isnan(m_att)), 'all') * config.dt_s;

  s_obj = config.s_obj + obj; % Sum of objective over time  

end

% Problems
% - m_att is different for each agent. Do we sum it over all agents?
% - 

% TODO
% - run simulation using mirko objective and show how it does not contribute
% towards 
% - check if we can simplify by re-using previous m_att calculations
% - add line to sum m_att across agents