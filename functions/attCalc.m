%% Function attcalc()
% Find attraction of cell

function [att] = attcalc(fis, scan_state, schedule_state, t_response, p_prior, t_dw)
  if scan_state || schedule_state
   att = NaN;
  else
    att = evalfis(fis, [t_response, p_prior, t_dw]);
  end
end