% V2

function config = i_sim_comms_disabled_victim_model_short()

%% Simulation Settings

% Data export configuration
flag_save    = true;
save_dir     = "simulations";

%% Time steps and counters
t       = 0;        % Current time
t_f     = 5000;    % Simulation end time (s)
dt_s    = 15;        % Simulation step size

% Discrete steps
dk_a    = 1;        % Agent step size
dk_c    = 1;        % Control step size
dk_e    = 4;        % Fire step size NOTE: environment model assumes 60s
dk_mpc  = 20;     % MPC step size
dk_prog = 50;     % Progress report step size

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

% Simulation end condition - "time" or "s can"
endCondition  = "time"; 
flag_finish    = false;

% Define agent objectives
weight = struct();
weight.dw = 0.1;      % Weight for victims
weight.fire = 0.1;      % Weight for victims
weight.first_scan = 1;   % Weight for the first-time scan. WARNING: IF 0, AGENT BEHAVIOUR COMPROMISED (will re-scan starting cells continuously)
weight.repeat_scan = 0.001;  % Weight for repeat scans

% Activate/deactivate victim model in calculation of priority
% This influences the assignment of cells to agents
flag_communication_model = false;

% Activate/deactivate discrete victim locations
% This influences the priority and objective calculations
flag_victim_model = true; 

% Objective function
obj           = 0;
s_obj         = 0;

config = struct('flag_save', flag_save, 'save_dir', save_dir, ...
  't', t, 't_f', t_f, 'dt_s', dt_s, 'dk_a', dk_a, 'dk_c', dk_c, 'dk_e', dk_e, 'dk_mpc', dk_mpc, 'dk_prog', dk_prog, 'dt_a', dt_a, 'dt_c', dt_c, 'dt_e', dt_e, 'dt_mpc', dt_mpc, ...
  'k', k, 'k_a', k_a, 'k_c', k_c, 'k_e', k_e, 'k_mpc', k_mpc, 'k_prog', k_prog, 'endCondition', endCondition, 'flag_finish', flag_finish, ...
  'obj', obj, 's_obj', s_obj, 'weight', weight, 'flag_communication_model', flag_communication_model, 'flag_victim_model', flag_victim_model);

end