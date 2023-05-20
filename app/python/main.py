import os
import time
import datetime
import numpy as np
from functions.plotting import *
from functions.report_progress import *
from functions.simulation_functions import *
import models as mod
from initialization import *

# Define function handles
# Function handles are used to refer to the different scripts used in the simulations
# Simulation variables - different mpc step sizes
h_init_sim_10 = initialise_simulation_10
# h_init_sim_50 = initialise_simulation_50
# h_init_sim_100 = initialise_simulation_100
# h_init_sim_200 = initialise_simulation_200

h_init_env_1 = initialise_environment_01
# h_init_env_2 = initialise_environment_02

# Agent
h_init_agt_1 = initialise_agent_01
# h_init_agt_2 = initialise_agent_02
# h_init_agt_3 = initialise_agent_03

# Path planning
h_init_pp_1 = initialise_pathPlanning_01
# h_init_pp_2 = initialise_pathPlanning_02

# MPC not active
h_init_MPC_1 = initialise_MPC_01

# MPC with optTermCond = 'MaxFunctionEvaluations'; / n_p = 1
h_init_MPC_maxfunceval_10 = initialise_MPC_maxfunceval_10
# h_init_MPC_maxfunceval_50 = initialise_MPC_maxfunceval_50
# h_init_MPC_maxfunceval_100 = initialise_MPC_maxfunceval_100
# h_init_MPC_maxfunceval_10_2 = initialise_MPC_maxfunceval_10_2
# h_init_MPC_maxfunceval_50_2 = initialise_MPC_maxfunceval_50_02
# h_init_MPC_maxfunceval_100_2 = initialise_MPC_maxfunceval_100_02

# Handles for solver test
h_init_MPC_SOLV_fminsearch = initialise_MPC_ST01_fminsearch
h_init_MPC_SOLV_ga = initialise_MPC_ST01_ga
h_init_MPC_SOLV_particleswarm = initialise_MPC_ST01_particleswarm
h_init_MPC_SOLV_patternsearch = initialise_MPC_ST01_patternsearch

if not multiSim:
    numIterations = 1

for iteration in range(numIterations):
    # Set seed for this iteration
    seed = int(time.mktime(datetime.datetime.now().timetuple())) if multiSim else 0

    for sim in range(np.shape(simulation_set)[0]):
        # Export folder for simulation
        simulation_name = simulation_set[sim, 0]

        # Initialize simulation data
        # Your initialization code goes here
        # ...

        # Initialize models
        # Your initialization code goes here
        # ...

        # Initialize plotting data
        # Your initialization code goes here
        # ...

        # Define timestep for saving variables
        # Your initialization code goes here
        # ...

        # Test setup
        # Your initialization code goes here
        # ...

        # Simulation
        while not flag_finish:
            # Start timer
            t_sim = time.time()

            # MPC
            # Your MPC code goes here
