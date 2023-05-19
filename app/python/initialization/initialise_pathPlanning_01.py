import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# NOTE: defining rules in python is different

# One thing to note is that the rule definition in Python's scikit-fuzzy library works differently than in MATLAB's FIS. In MATLAB, you can directly define a matrix of rules, but in Python, you have to define rules individually. This means that the ruleList needs to be translated into individual rule definitions in Python.

# Here is an example of how you might define rules in Python:

# rule1 = ctrl.Rule(antecedent=(
#     (name_in[0]['low'] & name_in[1]['low'] & name_in[2]['low']),
#     consequent=name_out['output']['low'])
# fis.add_rule(rule1)

# The above code creates a rule that says "if input1 is low and input2 is low and input3 is low, then the output is low". This rule is then added to the fuzzy system.

# You will need to define similar rules for all the rules in your ruleList.

def initialise_pathPlanning_01(m_bo_s, n_a):
    # Priority map 
    c_prior_building  = 1    # Priority constant for building
    c_prior_open      = 0.1  # Priority constant for open space

    # Calculate Priority map
    m_prior = c_prior_building * m_bo_s + c_prior_open * (1 - m_bo_s)

    # Generate FIS
    n_in            = 3
    n_out           = 1
    n_MF_in         = [3, 3, 3]
    n_MF_out        = [1]

    name_in         = ["t_response", "priority", "t_dw"]
    name_out        = ["attraction"]

    range_in        = [ [0, 1], 
                        [0, 1], 
                        [0, 1]]
    range_out       = [0, 1]

    MF_shape_in     = ["trimf", "trimf", "trimf"]
    MF_shape_out    = ["linear"]

    MF_range_in     = [[-0.1, 0.25, 0.5], 
                       [0.25, 0.5, 0.75], 
                       [0.5, 0.75, 1.1]]
    MF_range_out    = [-1, 1, -1, 1]

    MF_name_in      = [ ["low", "medium", "high"],
                        ["low", "medium", "high"],
                        ["low", "medium", "high"]]
                    
    MF_name_out     = ["output"]

    # ruleList to be defined, because in Python we use other way of defining rules

    fisArray = []

    # Generate FIS
    for a in range(n_a):
        # Create new FIS object
        fis = ctrl.ControlSystem()

        # Inputs
        for i_in in range(n_in):
            name_in_i_in = ctrl.Antecedent(np.arange(*range_in[i_in], 0.01), name_in[i_in])
            for i in range(n_MF_in[i_in]):
                name_in_i_in[MF_name_in[i_in][i]] = fuzz.trimf(name_in_i_in.universe, MF_range_in[i])

        # Outputs
        for i_out in range(n_out):
            name_out_i_out = ctrl.Consequent(np.arange(*range_out, 0.01), name_out[i_out])
            for i in range(n_MF_out[i_out]):
                # Because of linear membership function, we need two points only
                name_out_i_out[MF_name_out[i_out][i]] = fuzz.trimf(name_out_i_out.universe, MF_range_out[i:i+2])

        # Rule base to be defined, because in Python we use other way of defining rules

        fis.add(name_in_i_in)
        fis.add(name_out_i_out)

        fisArray.append(fis)
        
    return c_prior_building, c_prior_open, m_prior, fisArray, n_MF_out
