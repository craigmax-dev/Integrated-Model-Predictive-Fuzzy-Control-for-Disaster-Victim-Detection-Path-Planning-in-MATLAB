% Created: 22/07/2020
% Author: Craig Maxwell
% Create 4d scatter plot of FIS variable scaling data
function [] = scalingPlot(data)

  figure;
  
  % Data
  x = data(:,1);
  y = data(:,2);
  z = data(:,3);
  c = data(:,4);

  % Plot
  scatter3(x, y, z, 2, c);
  c = colorbar;

  % Labels
  xlabel('Nextcell Time')
  ylabel('Priority')
  zlabel('Downwind Time')
  c.Label.String = 'Attraction';
  
end