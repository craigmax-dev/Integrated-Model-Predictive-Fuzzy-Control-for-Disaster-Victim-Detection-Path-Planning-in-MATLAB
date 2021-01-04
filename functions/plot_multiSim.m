% Plot simulations individually and save to separate folder

% TO DO:
% Add functionality for relative and variable plots - currently always does
% relative
% Other ways of visualising data

%% Take average of results and plot - need to reformat plot_simulationComparisons function
% Note - not the most efficient method, but had to reduce complexity - just
% wanted results
axis_t = [];
maxsize = 1;
numIterations = 10;
numSimulations = 2;
simulation_set = "SS07";
simulation_set_names = [
    "SS07-1";
    "SS07-2";
    ];
plots_simSet = [
  "obj_hist", "variable";
  "obj_hist", "relative";
  "s_obj_hist", "variable";
  "s_obj_hist", "relative";
  ];


for plt = 1:size(plots_simSet, 1)
  data_name = plots_simSet(plt, 1);
  data_type = plots_simSet(plt, 2);
  for i=1:numIterations
    for j=1:size(simulation_set_names,1)
      sim_name = strcat(simulation_set_names(j), "_iter_", num2str(i));
      sim_route = strcat(exp_dir, ...
                    "\", simulation_set, ...
                    "\", sim_name, ...
                    "\", sim_name, ".mat");
      s_obj_hist_curr = load(sim_route, data_name);
      data = getfield(s_obj_hist_curr, data_name);
      datasize = size(data, 2);
      if(size(data, 2) > maxsize)
        t_data = load(sim_route, "t_hist");
        t_data = getfield(t_data, "t_hist");
        maxsize = datasize;
      end    
    end
  end

  s_obj_hist_mat = zeros(numIterations, numSimulations, maxsize);

  for i=1:numIterations
    for j=1:size(simulation_set_names,1)
      sim_name = strcat(simulation_set_names(j), "_iter_", num2str(i));
      sim_route = strcat(exp_dir, ...
                    "\", simulation_set, ...
                    "\", sim_name, ...
                    "\", sim_name, ".mat");
      s_obj_hist_curr = load(sim_route, data_name);
      data = getfield(s_obj_hist_curr, data_name);
      sizediff = maxsize - size(data, 2);
      if sizediff > 0
        data = [data, data(1,size(data,2))*ones(1,sizediff)];
      end    
      s_obj_hist_mat(i,j,:) = data;
    end
  end

  for i=1:numIterations
    s_obj_hist_mat_rel(i,:,:) = s_obj_hist_mat(i,:,:)./s_obj_hist_mat(i,1,:);
  end

  s_obj_hist_avg = squeeze(sum(s_obj_hist_mat, 1)/numIterations)';
  s_obj_hist_avg_rel = s_obj_hist_avg./s_obj_hist_avg(:,1);

  % All simulations
  figure; hold on; grid on;
  % Average
  plot(t_data, s_obj_hist_avg_rel(:,2), '--', 'LineWidth', 1.5);
  for i=1:numIterations
    data = squeeze(s_obj_hist_mat_rel(i,2,:));
    plot(t_data, data, 'LineWidth', 1.5);
  end

  lab_legend = [
    "Average"; 
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
  
%   lab_legend = [
%     "$$n_{\text{mpc}} = 10$$";
%     ];
  
  legend(lab_legend);
  legend('boxoff');
  axis([0 8000 -inf inf])
  [~, lab_x, lab_y, ~, ~, pos, ylimits] = func_plot_labels(data_name, data_type);
  xlabel(lab_x, 'Interpreter', 'latex');
  y = ylabel(lab_y, 'Interpreter', 'latex', 'rotation', 0);
  set(y, 'position', get(y,'position')-[200,0,0]); 

  % Average
  figure; hold on; grid on;
  % Average
  plot(t_data, s_obj_hist_avg_rel(:,2), '--', 'LineWidth', 1.5);

  lab_legend = "Average";
  legend(lab_legend);
  legend('boxoff');
  axis([0 8000 -inf inf])
  lab_x = '$t(s)$';
  lab_y = '$$\frac{\sum_{k=0}^{k_{f}}J_{n}}{\sum_{k=0}^{k_{f}}J_{1}}$$';
  xlabel(lab_x, 'Interpreter', 'latex');
  y = ylabel(lab_y, 'Interpreter', 'latex', 'rotation', 0);
  set(y, 'position', get(y,'position')-[200,0,0]); 

end
  