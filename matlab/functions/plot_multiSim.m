% Plot simulations individually and save to separate folder

%% Take average of results and plot - need to reformat plot_simulationComparisons function
% Note - not the most efficient method, but had to reduce complexity - just
% wanted results
close all;
axis_t = [];
maxsize = 1;
numIterations = 10;
lineStyles = [":", "-", "--"];
pos = [0.8, 0.6, 0.1, 0.1];
xlimits = [0, 7500];
[cmap, ~] = func_plot_colormaps('', true, numIterations);
% Data point offsets
offset_x = 100;
offset_y = 0;
xAdjust = 0.06;
xLabAdjust = [400,0,0];
% Make sure default formatter is latex
set(groot, 'defaultLegendInterpreter','latex');
opengl software

simulation_set = "SS08";
simulation_set_names = [
    "SS08-1";
    "SS08-2";
    "SS08-3";
    ];
numSimulations = length(simulation_set_names);
plots_simSet = [
  "obj_hist", "variable";
  "obj_hist", "relative";
  "s_obj_hist", "variable";
  "s_obj_hist", "relative";
  ];
lab_legend_iter = [
  "Sim 1"; 
  "Sim 2"; 
  "Sim 3"; 
  "Sim 4"; 
  "Sim 5"; 
  "Sim 6";
  "Sim 7"; 
  "Sim 8"; 
  "Sim 9"; 
  "Sim 10"; 
  ];
lab_legend_avg = [...
  "Average 1", ...
  "Average 2", ...
  "Average 3"];
exp_dir = "simulations";
exp_folder = "SS08";

for plt = 1:size(plots_simSet, 1)
  % Get maximum data size
  dataName = plots_simSet(plt, 1);
  dataType = plots_simSet(plt, 2);
  for i=1:numIterations
    for j=1:numSimulations
      sim_name = strcat(simulation_set_names(j), "_iter_", num2str(i));
      sim_route = strcat(exp_dir, ...
                    "\", sim_name, ...
                    "\", sim_name, ".mat");
      data_struct = load(sim_route, dataName);
      data_raw = getfield(data_struct, dataName);
      dataSize = size(data_raw, 2);
      if(size(data_raw, 2) > maxsize)
        t_data = load(sim_route, "t_hist");
        t_data = getfield(t_data, "t_hist");
        maxsize = dataSize; 
      end    
    end
  end
  
  data = NaN(maxsize, numSimulations, numIterations);

  % Load data
  for i=1:numSimulations
    for j=1:numIterations
      sim_name = strcat(simulation_set_names(i), "_iter_", num2str(j));
      sim_route = strcat(exp_dir, ...
                    "\", sim_name, ...
                    "\", sim_name, ".mat");
      data_struct = load(sim_route, dataName);
      data_raw = getfield(data_struct, dataName);
      data(1:length(data_raw),i,j) = data_raw;
    end
  end

  % Mean data and relative data
  data_mean = mean(data, 3);
  data_mean_rel = data_mean./data_mean(:, 1);
  for i=1:numIterations
    data_rel(:,:,i) = data(:,:,i)./data(:,1,i);
  end
  
  % Plot figures  
  for avgOrIter = 1:2
    if avgOrIter == 1
      figName = strcat(dataName, "_", dataType, "_", "avg");
      if dataType == "variable"
        plotData = data_mean;
      elseif dataType == "relative"
        plotData = data_mean_rel;
      end
    elseif avgOrIter == 2
      figName = strcat(dataName, "_", dataType, "_", "iter");
      if dataType == "variable"
        plotData = data;
      elseif dataType == "relative"
        plotData = data_rel;
      end
    end
    if dataName == "obj_hist" && dataType == "relative"
      ylimits = [-inf 2];
    else
      ylimits = [-inf inf];
    end

    figure("name", figName); hold on; grid on;

    if avgOrIter == 1
      % Average plots
      lab_legend = lab_legend_avg;
      for i = 1:numSimulations
        % Plot data
        plot(t_data, plotData(:, i), 'color', cmap(2*i), 'LineStyle', "-", 'LineWidth', 1.5);    
        % Label last value in plot
        numNans = sum(isnan(plotData(:, i)));
        finalPos = length(plotData(:, i)) - numNans;
        finalVal = plotData(finalPos, i);
        if ~isinf(finalVal)
          text(t_data(finalPos)+offset_x, finalVal+offset_y, num2str(finalVal))            
        end
      end
    elseif avgOrIter == 2
      % Iteration plots
      lab_legend = lab_legend_iter;
      for i = 1:numSimulations
        if i > 1
          for j=1:numIterations
            plot(t_data, plotData(:, i, j), 'color', cmap(j), 'LineStyle', lineStyles(i), 'LineWidth', 1.5);
          end
        end
      end
    end
    
    % Legend and axis
    [~, lab_x, lab_y, ~, ~, ~, ~] = func_plot_labels(dataName, dataType);
    xlabel(lab_x, 'Interpreter', 'latex');
    xlim(xlimits);
    ylim(ylimits);
    y = ylabel(lab_y, 'Interpreter', 'latex', 'rotation', 0);
    ax = gca();
    ax.OuterPosition(1) = xAdjust;
    set(y, 'position', get(y,'position')-xLabAdjust);         
    legend(lab_legend, 'position', pos, 'Interpreter', 'latex');
    legend('boxoff');
 
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
end
  