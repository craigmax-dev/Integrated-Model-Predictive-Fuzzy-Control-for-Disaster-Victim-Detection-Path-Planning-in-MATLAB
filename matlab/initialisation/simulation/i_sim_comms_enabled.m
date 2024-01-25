% V2


% CHANGELOG
% Refactor: config struct

% TODO: 
% - remove test_fis_sensitivity flags
% - check where r_bo, r_fo are used
% Remove: exp_dir, flag_fig_simSet, flag_fig_sim, flag_data_exp, test_solvers,
% test_fis_sensitivity, dt_c, dt_mpc, 

function config = i_sim_comms_enabled()

%% Simulation Settings
% Tests 
test_fis_sensitivity  = false;
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
t_f     = 40000;    % Simulation end time (s)
dt_s    = 15;        % Simulation step size

% Discrete steps
dk_a    = 1;        % Agent step size
dk_c    = 1;        % Control step size
dk_e    = 4;        % Fire step size NOTE: environment model assumes 60s
dk_mpc  = 10;       % MPC step size
dk_prog = 1000;      % Progress report step size

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

% Define agent objectives
weight = struct();
weight.dw = 0;      % Weight for victims
weight.fire = 0;      % Weight for victims
weight.first_scan = 0.5;   % Weight for the first-time scan
weight.repeat_scan = 0.1;  % Weight for repeat scans

communication_enabled = true;

% Objective function
obj           = 0;
s_obj         = 0;

% Risk model
r_bo  = 0.5;      % Risk weighting due to building occupancy
r_fo  = 0.5;      % Risk weighting due to environmental fire

config = struct('test_fis_sensitivity', test_fis_sensitivity, 'test_solvers', test_solvers, 'fis_data', fis_data, ...
  'flag_data_exp', flag_data_exp, 'flag_fig_sim', flag_fig_sim, 'flag_fig_simSet', flag_fig_simSet, 'exp_dir', exp_dir, ...
  't', t, 't_f', t_f, 'dt_s', dt_s, 'dk_a', dk_a, 'dk_c', dk_c, 'dk_e', dk_e, 'dk_mpc', dk_mpc, 'dk_prog', dk_prog, 'dt_a', dt_a, 'dt_c', dt_c, 'dt_e', dt_e, 'dt_mpc', dt_mpc, ...
  'k', k, 'k_a', k_a, 'k_c', k_c, 'k_e', k_e, 'k_mpc', k_mpc, 'k_prog', k_prog, 'endCondition', endCondition, 'flag_finish', flag_finish, ...
  'obj', obj, 's_obj', s_obj, 'r_bo', r_bo, 'r_fo', r_fo, 'weight', weight, 'communication_enabled', communication_enabled);

end