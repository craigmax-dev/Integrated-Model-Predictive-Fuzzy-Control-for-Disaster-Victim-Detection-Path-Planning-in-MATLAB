% Plot simulations individually and save to separate folder

% TO DO:
% get t axis values - currently has none

% numIterations = 10;
% for i=1:numIterations
%   simulation_set = {
%     "SS07-1", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_1;
%     "SS07-2", h_init_sim_50, h_init_env_1, h_init_agt_1, h_init_pp_2, h_init_MPC_maxfunceval_50_2;
%     };
%   simulation_set_names = [
%     "SS07-1";
%     "SS07-2";
%     ];
%   for j=1:size(simulation_set,1)
%     simulation_set{j, 1} = strcat(simulation_set{j, 1}, "_iter_", num2str(i));
%     simulation_set_names(j) = strcat(simulation_set_names(j), "_iter_", num2str(i));
%   end
%   % Plot settings
%   plots_simSet = { 
%     "obj_hist", "variable", true;
%     "obj_hist", "relative", true;
%     "s_obj_hist", "variable", true;
%     "s_obj_hist", "relative", true;
%     "fis_param_hist", "fis", true;
%     };
%   % Plotting function
%   plot_simulationComparisons(plots_simSet, exp_dir, simulation_set, simulation_set_name, simulation_set_names);
% end


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
for i=1:numIterations
  for j=1:size(simulation_set_names,1)
    % Load simulation
    % Note: this adds zeros to array - will skew the results
    % Find better method
    sim_name = strcat(simulation_set_names(j), "_iter_", num2str(i));
    sim_route = strcat(exp_dir, ...
                  "\", simulation_set, ...
                  "\", sim_name, ...
                  "\", sim_name, ".mat");
    s_obj_hist_curr = load(sim_route, 's_obj_hist');
    data = getfield(s_obj_hist_curr, "s_obj_hist");
    datasize = size(data, 2);
    if(size(data, 2) > maxsize)
      t_data = ...
      axis_t = t_data;
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
    s_obj_hist_curr = load(sim_route, 's_obj_hist');
    data = getfield(s_obj_hist_curr, "s_obj_hist");
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

figure; hold on; grid on;
% data = squeeze(s_obj_hist_mat_rel(1,1,:));
% plot(data, 'LineWidth', 1.5);
% Average
plot(s_obj_hist_avg_rel(:,2), '--', 'LineWidth', 1.5);
for i=1:numIterations
  data = squeeze(s_obj_hist_mat_rel(i,2,:));
  plot(data, 'LineWidth', 1.5);
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
legend(lab_legend);
legend('boxoff');