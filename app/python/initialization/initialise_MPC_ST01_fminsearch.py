import numpy as np
from scipy.optimize import fminsearch, OptimizeResult


def initialise_MPC_ST01_fminsearch(fisArray, n_a):
    flag_mpc = True
    # Solver options: fminsearch (equivalent to fmincon), ga, particleswarm, patternsearch
    solver = fminsearch
    # Prediction horizon
    n_p = 1
    # Initialise optimisation parameters
    fis_params = []
    for a in range(n_a):
        fis_params.extend(fisArray[a].Outputs.MembershipFunctions.Parameters)
    ini_params = []
    for _ in range(n_p):
        ini_params.extend(fis_params)
    # Constraints
    A = []
    b = []
    Aeq = []
    beq = []
    lb = []
    ub = []
    nonlcon = []
    nvars = len(ini_params)
    # Solver options
    optTermCond = "MaxTime"
    optTermCond_value = 300
    options = {"disp": True, optTermCond: optTermCond_value}

    return (
        flag_mpc,
        solver,
        options,
        n_p,
        fis_params,
        ini_params,
        A,
        b,
        Aeq,
        beq,
        lb,
        ub,
        nonlcon,
        nvars,
    )
