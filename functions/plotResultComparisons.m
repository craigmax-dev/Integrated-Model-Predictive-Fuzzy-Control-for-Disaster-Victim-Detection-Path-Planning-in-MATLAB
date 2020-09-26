% Function to compare results from simulations

%% To do
% fis_param_hist scatter
% Automate import of simulation data for comparison (by defining file names?)

%% Plots
% % fis_param_hist scatter

% obj_sum_hist
data_name = ["SC0.1", "SOPAT0.2", "SOPAT0.3"];
data_t_end = [9340, 6820, 6460];

h = figure('name','s_obj_hist_comparison');
hold on;
grid on;
xlabel("Time (s)"); 
ylabel("Objective function sum");
title("Objective function sum over time comparison");

for i = 1:length(data)
  t_v = linspace(0, data_t_end(i), size(data{i},2));
  plot(t_v, data{i})
end

legend(data_name)

% obj_hist
data_name = ["SC0.1", "SOPAT0.2", "SOPAT0.3"];
data_t_end = [9340, 6820, 6460];
t_v = linspace(0, t_end, ct_v);

h = figure('name','obj_hist_comparison');
hold on;
grid on;
xlabel("Time (s)"); 
ylabel("Objective function");
title("Objective function over time comparison");

for i = 1:length(data)
  t_v = linspace(0, data_t_end(i), size(data{i},2));
  plot(t_v, data{i})
end

legend(data_name)