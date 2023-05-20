%% Function attCalc()
% Return attraction of cell

function [att] = calc_att(fis, t_response, p_prior, t_dw, scan_state, schedule_state)
  if scan_state || schedule_state
   att = NaN;
  else
    att = evalfis(fis, [t_response, p_prior, t_dw]);
  end
end