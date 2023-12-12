% V2

function [test_fis_sensitivity, test_obj_sensitivity, test_solvers, fis_data, ...
  flag_data_exp, flag_fig_sim, flag_fig_simSet, exp_dir, ...
  t, t_f, dt_s, dk_a, dk_c, dk_e, dk_mpc, dk_prog, dt_a, dt_c, dt_e, dt_mpc, ...
  k, k_a, k_c, k_e, k_mpc, k_prog, endCondition, flag_finish, ...
  obj, s_obj, r_bo, r_fo] = initialise_simulation_SIM_1()

%% Simulation Settings
% Tests 
test_fis_sensitivity  = false;
test_obj_sensitivity  = false;
test_solvers          = false;
% Test variables
fis_data = [];
% Data export configuration
flag_data_exp    = true;
flag_fig_sim     = true;
flag_fig_simSet  = true;
exp_dir     = "simulations";

%% Time steps and counters
t       = 0;        % Current time
t_f     = 18000;    % Simulation end time (s)
dt_s    = 30;        % Simulation step size

% Discrete steps
dk_a    = 1;        % Agent step size
dk_c    = 2;        % Control step size
dk_e    = 2;       % Fire step size
dk_mpc  = 10;       % MPC step size
dk_prog = 10;      % Progress report step size

% Time steps
dt_a    = dk_a*dt_s;
dt_c    = dk_c*dt_s;
dt_e    = dk_e*dt_s;
dt_mpc  = dk_mpc*dt_s;

k       = 0;        % Discrete time step counter
k_a     = 0;        % Agent counter
k_c     = 0;        % Control counter
k_e     = 0;        % Fire counter
k_mpc   = 0;        % MPC counter
k_prog  = 0;        % Prog report counter

% Simulation end condition - "time" or "scan"
endCondition  = "time"; 
flag_finish    = false;

% Objective function
obj           = 0;
s_obj         = 0;

% Risk model
r_bo  = 0.5;      % Risk weighting due to building occupancy
r_fo  = 0.5;      % Risk weighting due to environmental fire

end