% Function to compare results from simulations
function [] = plot_simulationComparisons(plots_simSet, exp_dir, ...
  simulation_set, simulation_set_name, simulation_set_names)

% Plotting improvements

% - cell scan times
% - cell scan time differences between control simulation and current simulation
% - fire map

% Simulation specific
% - Simulation 4 - compare results from one output to two output surfaces

% TO DO LIST
% Not saving properly
% plotting/marking of prediction horizon length?
% remove _iter_ naming of export folders when multisim not active
% x axis - change nomenclature - use variable defining simulation time.
% re-run simulation set 1 - results not correct for some reason?
% limit number of decimal places for plot end values?
% Move legend location definitions to argument for plotting function?

  % Close any open figures
  close all
  % Make sure default formatter is latex
  set(groot, 'defaultLegendInterpreter','latex');
  opengl software
  % Data point offsets
  offset_x = 100;
  offset_y = 0;
  xAdjust = 0.06;
  xLabAdjust = [400,0,0];
  maxsize = 1;

  %% Plotting
  % Iterate through each plot in set
  for i = 1:size(plots_simSet,1)
    % Load data information
    dataName = plots_simSet{i,1};
    dataType = plots_simSet{i,2};
    flag_plot = plots_simSet{i,3};
    % Labels
    lab_title = '';
    lab_x = '';
    lab_y = '';
    % Number of simulations
    numSimulations = size(simulation_set,1);
    % Retrieve desired labels from label function
    [~, lab_x, lab_y, ~, ~, pos, ylimits] = func_plot_labels(dataName, dataType);
    % Color map
    [cmap, ~] = func_plot_colormaps('', true, numSimulations);
    if flag_plot

      for sim=1:numSimulations
        sim_name = simulation_set{sim, 1};        
        sim_route = strcat(exp_dir, ...
                              "\", sim_name, ...
                              "\", sim_name, ".mat");
        data_struct = load(sim_route, dataName);

        data_struct = load(sim_route, dataName);
        data_raw = getfield(data_struct, dataName);
        dataSize = size(data_raw, 2);
        if(size(data_raw, 2) > maxsize)
          t_data_raw = load(sim_route, "t_hist");
          t_data = getfield(t_data_raw, "t_hist");
          maxsize = dataSize; 
        end    
      end

      data = NaN(maxsize, numSimulations);

      % Load data into matrix
      for sim = 1:numSimulations
        sim_name = simulation_set{sim, 1};        
        sim_route = strcat(exp_dir, ...
                              "\", sim_name, ...
                              "\", sim_name, ".mat");
        data_struct = load(sim_route, dataName);
        data_raw = getfield(data_struct, dataName);
        data(1:length(data_raw),sim) = data_raw;
      end
      
      % Mean data and relative data
      for sim=1:numSimulations
        data_rel(:,sim) = data(:,sim)./data(:,1);
      end
      
      if dataType == "variable"
        plotData = data;
      elseif dataType == "relative"
        plotData = data_rel;
      end
      
      % Plot figure
      figName = strcat(dataName, "_", dataType);
      figure("name", figName); hold on; grid on;
      
      for sim = 1:numSimulations
        if numSimulations == 2
          lineStyle = "-";
          colour = sim;
        elseif sim <= floor(numSimulations/2)+1
          lineStyle = "-";
          colour = sim;
        else
          lineStyle = "--";
          colour = sim - (floor(numSimulations/2));
        end
        % Plot each row
        plot(t_data, plotData(:,sim), 'LineWidth', 1.5, 'color', cmap(colour), 'LineStyle', lineStyle);
        if dataType == "relative"
          % Label last value in plot
          numNans = sum(isnan(plotData(:, sim)));
          finalPos = length(plotData(:, sim)) - numNans;
          finalVal = plotData(finalPos, sim);
          if ~isinf(finalVal)
            text(t_data(finalPos)+offset_x, finalVal+offset_y, num2str(finalVal))            
          end
        end
      end
      % Labels
      title(lab_title);
      ylim(ylimits);
      xlabel(lab_x, 'Interpreter', 'latex');
      y = ylabel(lab_y, 'Interpreter', 'latex', 'rotation', 0);
      set(y, 'position', get(y,'position')-xLabAdjust);
      ax = gca();
      ax.OuterPosition(1) = xAdjust;
      legend(simulation_set_names, 'position', pos, 'Interpreter', 'latex');
      legend('boxoff');
    end
  end

  %% Save figures as .fig and .jpg files
  fig_list = findobj(allchild(0), "Type", "figure");
  for iFig = 1:length(fig_list)
    h_fig = fig_list(iFig); 
    figName   = get(h_fig, "Name");
  %   % Export directory address
    exp_folder = strcat(exp_dir, "/", simulation_set_name);
    exp_fig = strcat(exp_folder, "\", figName);
    % Create save directory
    if(~exist(exp_folder, 'dir'))
      mkdir(exp_folder)
    end
    exportgraphics(h_fig, strcat(exp_fig, ".jpg"));
    close(h_fig);
  end
end

%% Initial attempt at plotting fis params at each mpc step 
% % - need to think about how to do this
%         % Iterate through each simulation in set
%         for mpc = 1:size(data_obj{2},1)
%           % Figure name
%           fig_name = strcat(data_name, "_", num2str(mpc));
%           % Create figure
%           f = figure("name", fig_name);
%           hold on;
%           % For each mpc step
%           
%           
%           for sim = 1:size(simulation_set,1)
%             axis_x = 1:size(data_obj{1},2);
%             for mpc = 1:k_mpc
%               if sim == 1
%                 data = data_obj{sim};
%               else
%                 data = data_obj{sim}(mpc, :);
%               end
%               scatter(axis_x, data(mpc, :))              
%             end
%           end
%         end