%% Function attcalc()
% Find attraction of cell

%% Change Log
% 01/08/2020 - changed to use arrayfun()

function [att] = attcalc(fis, t_nextcell, prior, t_dw)
  if isnan(t_nextcell)
   att = NaN;
  else
    att = evalfis(fis, [t_nextcell, prior, t_dw]);
  end
end
