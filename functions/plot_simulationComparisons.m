% Function to compare results from simulations
function [] = plot_simulationComparisons(plots_simSet, exp_dir, ...
  simulation_set, simulation_set_name, simulation_set_names)

  % Close any open figures
  close all
  % Export directory address
  exp_dir = strcat(exp_dir, "/", simulation_set_name);
  % Create save directory
  if(~exist(exp_dir, 'dir'))
    mkdir(exp_dir)
  end
  % Color map
  [cmap, ~] = func_plot_colormaps('', true);
  % Make sure default formatter is latex
  set(groot, 'defaultLegendInterpreter','latex');
  opengl software
  %% Plotting
  % Iterate through each plot in set
  for i = 1:size(plots_simSet,1)
    % Load data information
    data_name = plots_simSet{i,1};
    data_type = plots_simSet{i,2};
    flag_plot = plots_simSet{i,3};
    % Labels
    lab_title = '';
    lab_x = '';
    lab_y = '';
    % Retrieve desired labels from label function
    [~, lab_x, lab_y, ~, ~, pos, ylimits,] = func_plot_labels(data_name, data_type);
    if flag_plot
      % Load data into matrix
      for sim = 1:size(simulation_set,1)
        % Get files
        sim_name = simulation_set{sim, 1};        
        sim_route = strcat(exp_dir, ...
                              "\", sim_name, ...
                              "\", sim_name, ".mat");
        data_struct = load(sim_route, data_name, "t_hist");
        % Extract data
        fields = fieldnames(data_struct);
        % Data array
        new_data = getfield(data_struct, fields{1});
        data_obj{sim} = new_data;
        % Time array
        new_axis = getfield(data_struct, fields{2});
        axis_t_obj{sim} = new_axis;
        % Legend
        lab_legend{sim} = sim_name;
      end
      % Array for legend
      lab_legend_arr = string(lab_legend);
      % Plot
      if data_type == "variable"
        % Figure name
        fig_name = data_name;
        % Create figure
        f = figure("name", fig_name);
        hold on;
        for sim = 1:size(simulation_set,1) 
          % Plot each row
          plot(axis_t_obj{sim}, data_obj{sim}, 'LineWidth', 2);
        end
      elseif data_type == "relative"
        % Change data name
        fig_name = strcat(data_name, "_relative");
        % Create figure
        f = figure("name", fig_name);
        hold on;
        % Change legend position
        pos = [0.7, 0.75, 0.1, 0.1];
        for sim = 1:size(simulation_set,1)
          data(sim, 1:numel(data_obj{sim})) = data_obj{sim};
          axis_t(sim, 1:numel(axis_t_obj{sim})) = axis_t_obj{sim};
        end
        % Relative
        data = data./data(1, :); 
        % Remove additional elements added to end of vector and plot each row
        for sim = 1:size(simulation_set,1)
          % Temp solution - define line styles for simulations
          if sim <= 4
            lineStyle = "-";
            colour = sim;
          else
            lineStyle = "--";
            colour = sim - 3;
          end
            
          curr_axis = axis_t(sim, :);
          curr_data = data(sim, :);
          trim_axis = curr_axis(1:find(curr_axis~=0,1,'last'));
          trim_data = curr_data(1:length(trim_axis));
          plot(trim_axis, trim_data, 'LineWidth', 1.5, 'LineStyle', lineStyle);
%           plot(trim_axis, trim_data, 'LineWidth', 1.5, 'color', cmap(colour), 'LineStyle', lineStyle);
        end
      elseif data_type == "fis"
        % Iterate through each simulation in set
        for sim = 1:size(simulation_set,1)
          % Figure name
          fig_name = strcat(data_name, "_", num2str(sim));
          % Create figure
          f = figure("name", fig_name);
          hold on;
          data = data_obj{sim};
          axis_x = 1:size(data,2);
          for mpc = 1:size(data,1)
            scatter(axis_x, data(mpc, :), 'LineWidth', 2)
          end
        end
      end
      grid on;
      ylim(ylimits);
      % Labels
      title(lab_title);
      xlabel(lab_x, 'Interpreter', 'latex');
      y = ylabel(lab_y, 'Interpreter', 'latex', 'rotation', 0);
      set(y, 'position', get(y,'position')-[200,0,0]); 
      legend(simulation_set_names, 'position', pos);
      legend('boxoff');
    end
  end

  %% Save figures as .fig and .jpg files
  fig_list = findobj(allchild(0), "Type", "figure");
  for iFig = 1:length(fig_list)
    h_fig = fig_list(iFig); 
    fig_name   = get(h_fig, "Name");
    exp_fig = strcat(exp_dir, "\", fig_name);
    exportgraphics(h_fig, strcat(exp_fig, ".jpg"));
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