# Integrated Model Predictive Fuzzy Control for Disaster Victim Detection Path Planning in MATLAB

**Please note currently the README file is out of date**

This is the code for my in-progress thesis project for my MSc in Control and Simulation, Aerospace Engineering at the Delft University of Technology.

<!-- :exclamation: This is a work in progress! :exclamation:
Please note that the code here is not complete. There currently may be various bugs and improvements that need to be made to the simulation. When the version of the code to be used in my thesis is complete, I will release it as a version in this repo.
 -->
 
This project consists of a simulation of a search-and-rescue environment for discrete path-planning of agents using a Fuzzy Inference System (FIS)-based controller and a Model Predictive Control (MPC)-based controller to optimise FIS parameters. 
The environment model consists of static states (building coverage and wind) and dynamic states (fire and agents). 
The inputs used for the FIS are distance, priority, and downwind time; each of which are modelled in the simulation. 
The output of the FIS is attraction which is used by the path-planner to decide on waypoint locations for the UAVs. 
The MPC uses an objective function which is a combination of the accumulative priority of unscanned cells and an additional priority due to cells which are on fire. 
The MPC uses the patternsearch solver for the nonlinear time-limited optimisation of the FIS parameters.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

* [MATLAB](https://www.mathworks.com/products/matlab.html) - Version: '9.8'
* [Mapping Toolbox]() - Version: '4.10'
* [Image Processing Toolbox]() - Version: '11.1'
* [Fuzzy Logic Toolbox](https://www.mathworks.com/products/fuzzy-logic.html) - Version: '2.7'
* [Global Optimization Toolbox]() - Version: '4.3'
* [Antenna Toolbox]() - Version: '4.2'
* [Curve Fitting Toolbox]() - Version: '3.5.11'
* [Fixed-Point Designer]() - Version: '7.0'
* [System Identification Toolbox]() - Version: '9.12'
* [MATLAB Coder]() - Version: '5.0'
* [Optimization Toolbox](https://www.mathworks.com/products/optimization.html) - Version: '8.5'
* [Simulink]() - Version: '10.1'
* [Statistics and Machine Learning Toolbox]() - Version: '11.7'

### Installing

This code can be installed in several easy steps.

1 - Download the project.

2 - Ensure required dependencies are installed.

3 - Run the main.m script to check the project is functional.

## Running the tests

Default tests will be included in the full release of the project code.

## Contributing

This is a thesis project as part of my final project for my MSc in Control and Simulation, Aerospace Engineering at the Delft University of Technology. 
Contributions will be welcome after the final version of the code for the thesis is produced. There are many options to continue building on and improving the code. 
Please read [CONTRIBUTING.md](https://gist.github.com/craigmax-dev/contributing) for details on our code of conduct, and the process for submitting pull requests to us. 
For inspiration or ideas of how to contribute to the project, please read [RECOMMENDATIONS.md](RECOMMENDATIONS.md).

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags](https://github.com/craigmax-dev/Integrated-Model-Predictive-Fuzzy-Control-for-Disaster-Victim-Detection-Path-Planning-in-MATLAB/tags). 

## Authors

* **Craig Maxwell** - *Initial work* - [craigmax-dev](https://github.com/craigmax-dev)
* **Mirko Baglioni** - Several adaptations are made from the PhD project of Mirko.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

## Function Descriptions (IN PROGRESS)

### `calculateAgentDistances` Function

The `calculateAgentDistances` function computes the minimum Euclidean distance from each grid cell to the nearest agent in a specified grid. The function is designed to operate in a grid environment where multiple agents are positioned, and the goal is to calculate the distance from each point on the grid to the closest agent.

#### Inputs

- `gridSize`: A two-element vector `[numRows, numCols]` specifying the number of rows and columns in the grid.
- `agentPositions`: An `n`-by-2 matrix, where `n` is the number of agents, and each row represents the `(x, y)` coordinates of an agent within the grid.

#### Process

1. **Initialization:**
   - Extract the number of rows (`numRows`) and columns (`numCols`) from `gridSize`.
   - Determine the number of agents (`numAgents`) from the dimensions of `agentPositions`.
   - Initialize `distanceMatrix`, a cell array where each cell will contain a matrix representing the minimum distances from all points in the grid to a particular agent.

2. **Distance Calculation:**
   - For each agent `a`, iterate over all grid cells `(row, col)`.
   - For each cell, calculate the Euclidean distance to all other agents `otherAgent`.
   - Update the distance matrix for agent `a` at position `(row, col)` with the minimum distance to any other agent:
     - The distance `distToOtherAgent` from a cell at `(row, col)` to an agent positioned at `(agentPositions(otherAgent, 1), agentPositions(otherAgent, 2))` is calculated using the Euclidean distance formula:
       $$ \text{distToOtherAgent} = \sqrt{(col - agentPositions(otherAgent, 1))^2 + (row - agentPositions(otherAgent, 2))^2} $$
     - The value in `distanceMatrix{a}(row, col)` is updated to the minimum of its current value and `distToOtherAgent`, ensuring it represents the closest distance to any agent.

#### Output

- `distanceMatrix`: A cell array of matrices, one per agent. Each matrix has dimensions `[numRows, numCols]`, and the value at each `(row, col)` represents the minimum Euclidean distance from that cell to the nearest agent, excluding the agent itself.

### `calculateFireSpreadLikelihood` Function

The `calculateFireSpreadLikelihood` function computes a map of the likelihood of fire spread within a specified environment and time period, taking into account wind influence and other environmental factors.

#### Inputs

- `environment_model`: A struct containing environmental parameters, including the dimensions of the environment (`n_x_e`, `n_y_e`), the fire status matrix (`m_f`), wind direction (`ang_w`), wind speed (`v_w`), and other relevant parameters.
- `config`: A struct containing configuration parameters, including the time step (`dt_e`) for the calculation.
- `time_period`: The total time period for which the fire spread likelihood is being calculated.

#### Process

1. **Initialization:**
   - Initialize `F_likelihood`, a matrix with dimensions corresponding to the environment (`n_x_e` by `n_y_e`), to zeros. This matrix will store the likelihood of fire spread to each cell.

2. **Wind Influence Matrix Calculation:**
   - Calculate `W`, the wind influence matrix, using `calculateWindSpreadMatrix` function with parameters from `environment_model`. `W` incorporates the effects of wind direction and speed on fire spread.

3. **Burn Time Window Assumption:**
   - For each time step `t` within the given `time_period`, iterate over all cells in the environment to calculate the fire spread likelihood.

4. **Fire Spread Likelihood Calculation:**
   - Skip cells that are already burnt or burning (`m_f(i, j) == 3` or `m_f(i, j) == 4`).
   - Calculate the spread probability `p` for the current time `t` using `calculateSpreadProbability`, which considers the ignition time (`t_i`) and burn time (`t_b`) for the environment.
   - Update the fire spread likelihood for the current cell using `updateFireSpreadProbability`, which takes into account the calculated probability `p`, wind influence `W`, and other environmental factors such as slope (`m_s`) and building occupancy (`m_bo`).

5. **Normalization:**
   - Normalize `F_likelihood` so that all values represent probabilities between 0 and 1. Values greater than 1 are set to 1.

#### Output

- `F_likelihood`: A matrix representing the likelihood of fire spread to each cell in the environment over the specified `time_period`, normalized to probabilities between 0 and 1.

### `calc_att` Function

The `calc_att` function calculates the attraction level for each cell in an environment, given the response time, priority, and the targets scheduled by agents. It considers whether communication among agents is enabled to avoid scheduling conflicts.

#### Inputs

- `fis`: The Fuzzy Inference System used to calculate attraction based on response time and priority.
- `m_t_response`: A matrix representing the response time for each cell.
- `m_prior`: A matrix representing the priority of each cell.
- `a_target`: A 3D array specifying the target locations for each agent at each queue position.
- `communication_enabled`: A boolean flag indicating whether agents can communicate their scheduled cells with each other.

#### Process

1. **Initialization:**
   - Initialize `m_att` with `NaN` values, having the same dimensions as `m_t_response`.

2. **Scheduled Cells Identification:**
   - Create a logical matrix `scheduled_cells` indicating which cells are already scheduled by agents. The process differs based on the `communication_enabled` flag:
     - **With Communication:** Consider cells scheduled by all agents.
     - **Without Communication:** Consider only cells scheduled by the agent in question.

3. **Valid Cells Determination:**
   - Identify valid cells for attraction calculation as those not scheduled (`~scheduled_cells`) and not containing `NaN` in `m_t_response` or `m_prior`.

4. **FIS Input Preparation:**
   - Extract `m_t_response` and `m_prior` for valid cells and combine them into a matrix `fisInputs`.

5. **Attraction Calculation:**
   - For each valid cell, calculate the attraction level using the FIS with `m_t_response` and `m_prior` as inputs. The resulting attraction levels are assigned to `m_att` for corresponding valid cells.

#### Output

- `m_att`: A matrix of the same dimensions as `m_t_response`, containing calculated attraction levels for each cell. Cells scheduled by agents or otherwise invalid for calculation retain their `NaN` initialization.

### `calc_maxResponseTime` Function

The `calc_maxResponseTime` function computes the maximum response time for an agent to cover the diagonal distance of a search area, factoring in the travel and scanning times across all queued tasks.

#### Inputs

- `l_x_s`, `l_y_s`: The scaling factors for distances along the x and y dimensions, representing the physical distance each cell covers.
- `n_q`: The number of tasks in the queue for an agent.
- `n_x_s`, `n_y_s`: The dimensions of the environment in terms of the number of cells along the x and y axes.
- `t_scan_c`: The constant scan time for each task.
- `v_as`: The airspeed of the agent in meters per second.

#### Calculations

1. **Total Length and Width of the Search Area:**
   - The total length and width of the search area are calculated based on the environment dimensions and scaling factors:
     - `totalLength = (n_x_s - 1) * l_x_s`
     - `totalWidth = (n_y_s - 1) * l_y_s`

2. **Diagonal Distance of the Search Area:**
   - The diagonal distance, representing the longest straight-line distance across the search area, is computed as:
     - `diagonalDistance = sqrt(totalLength^2 + totalWidth^2)`

3. **Maximum Response Time Calculation:**
   - The maximum response time considers the time to travel the diagonal distance at the agent's airspeed (`v_as`) and the total scan time for all queued tasks. It's computed as:
     - `maxResponseTime = (n_q + 1) * (diagonalDistance / v_as) + (n_q + 1) * t_scan_c`
   - This formula accounts for the travel and scan time for `n_q` tasks, plus an additional round for initial positioning.

#### Output

- `maxResponseTime`: The calculated maximum response time, which is the sum of the maximum possible travel time across the diagonal of the search area and the total scan time for all tasks in the queue, including an initial positioning round.

### `calc_obj` Function

The `calc_obj` function calculates an objective value based on various factors in a simulated environment, including the presence of fires, victim likelihood, and the timing of scans. It is designed to quantify the priority or urgency of scanning different areas, factored by the presence of fires and potential victims.

#### Inputs

- `weight`: A struct containing weights for different components (`fire`, `first_scan`, `repeat_scan`).
- `m_f`: A matrix representing the fire status in each cell (with a value of 3 indicating active fires).
- `m_bo_s`: A matrix representing the building occupancy at a coarse resolution, serving as a proxy for victim likelihood if no victim map is provided.
- `m_scan`: A matrix indicating the last scan time for each cell.
- `m_victim_s`: A matrix representing the likelihood of victims in each cell. If empty, `m_bo_s` is used as a proxy.
- `dt_s`: The simulation time step.
- `s_obj`: The current sum of the objective function over time.
- `c_f_s`: Coarsening factor for scaling environment maps to agent resolution.
- `t`: The current simulation time.

#### Process

1. **Active Fires Map (`m_fo`):**
   - Identify cells with active fires: `m_fo = (m_f == 3)`.

2. **Victim Parameter:**
   - Use `m_victim_s` to represent victim likelihood. If `m_victim_s` is empty, use `m_bo_s` as a proxy.

3. **Coarsen Environment Maps:**
   - Apply `func_coarsen` to `m_fo` using `c_f_s` to adjust the resolution to match agent resolution, resulting in `m_fo_s`.

4. **Active Fire Weight (`m_P_fire_weight`):**
   - Calculate the weight applied to cells with active fires: `m_P_fire_weight = m_fo_s * weight.fire`.

5. **First-time Scan Priority Component (`m_P_first_scan`):**
   - For cells not yet scanned (`m_scan == 0`), calculate the priority based on victim likelihood and fire presence:
     `m_P_first_scan = double(m_scan == 0) .* m_victim_s .* (1 + m_P_fire_weight) * weight.first_scan`.

6. **Repeat Scan Priority Component (`m_P_repeat_scan`):**
   - Calculate the priority for repeat scans based on time since last scan, victim likelihood, and fire presence:
     `m_P_repeat_scan = time_since_last_scan .* m_victim_s .* (1 + m_P_fire_weight) * weight.repeat_scan`,
   where `time_since_last_scan = max(t - m_scan, 0)`.

7. **Total Priority Map (`m_P`):**
   - Sum the first-time and repeat scan priorities to obtain the total priority map: `m_P = m_P_first_scan + m_P_repeat_scan`.

8. **Objective Function Calculation:**
   - Calculate the objective for the current timestep as the sum of all values in `m_P` multiplied by `dt_s`, ensuring the result is always positive:
     `obj = max(sum(m_P, 'all') * dt_s, 0)`.
   - Update the cumulative objective over time: `s_obj = s_obj + obj`.

#### Output

- `s_obj`: Updated sum of the objective function over time, reflecting the cumulative priority of scanning activities.
- `obj`: The objective value calculated for the current timestep.

### `calc_prior` Function

The `calc_prior` function computes the scanning priority for areas within a grid. It considers various factors such as building occupancy, downwind map values, scan history, current time, victim information, and specific weights for different priority factors.

#### Inputs

- $m_{bo}$: Matrix representing building occupancy, with higher values indicating more occupancy.
- $m_{dw}$: Downwind map matrix, where values closer to 0 indicate proximity to hazards like fire.
- $m_{scan}$: Matrix indicating the last scan time for each cell, with 0 indicating that the cell has not been scanned.
- $t$: Current time.
- $weight$: Struct containing weights for different components of priority calculation:
  - $weight_{\text{first\_scan}}$: Weight for first-time scans.
  - $weight_{\text{repeat\_scan}}$: Weight for re-scans.
  - $weight_{\text{dw}}$: Weight for downwind information in priority calculation.
- $m_{victim}$: Matrix indicating the likelihood of finding victims in each cell, normalized to [0, 1] if `flag_victim_model` is true.
- $flag_{\text{victim\_model}}$: Boolean indicating whether the victim model is used for re-scan priority.

#### Process

1. **Normalize Victim Information (Optional):**
   If `flag_victim_model` is true, normalize $m_{victim}$ to the range [0, 1]:
   $$ m_{\text{victim}} = \frac{m_{\text{victim}}}{\max(m_{\text{victim}})} $$

2. **First-time Scan Priority ($m_{P_{\text{first\_scan}}}$):**
   Priority for cells not yet scanned, based on building occupancy:
   $$ m_{P_{\text{first\_scan}}} = (\mathbf{1}_{\{m_{\text{scan}} = 0\}}) \cdot m_{bo} \cdot weight_{\text{first\_scan}} $$

3. **Re-scan Priority ($m_{P_{\text{re\_scan}}}$):**
   Based on time since last scan and, optionally, on victim information or building occupancy:
   $$ m_{P_{\text{re\_scan}}} = (\mathbf{1}_{\{m_{\text{scan}} \neq 0\}}) \cdot (m_{\text{variable}} \cdot weight_{\text{repeat\_scan}} \cdot (\max(t - m_{\text{scan}}, 0)) + 1) $$
   Where $m_{\text{variable}}$ is $m_{victim}$ if `flag_victim_model` is true, otherwise $m_{bo}$.

4. **Downwind Map Information ($m_{P_{\text{dw}}}$):**
   Priority based on hazard proximity, inversely related to downwind values:
   $$ m_{P_{\text{dw}}} = (1 - m_{dw}) \cdot weight_{\text{dw}} $$

5. **Overall Priority ($m_{\text{prior}}$):**
   Summing the calculated priorities to obtain the overall priority matrix:
   $$ m_{\text{prior}} = m_{P_{\text{first\_scan}}} + m_{P_{\text{re\_scan}}} + m_{P_{\text{dw}}} $$

#### Output

- $m_{\text{prior}}$: Matrix of priorities for scanning cells, where higher values indicate a higher priority.

### `calc_t_response` Function

The `calc_t_response` function calculates the response time matrix for agents to reach various target locations in an environment, considering the agents' current tasks, travel times, and scan times.

#### Inputs

- `n_x_s`, `n_y_s`: Dimensions of the environment in terms of cells along the x and y axes.
- `l_x_s`, `l_y_s`: Scaling factors for distances along the x and y dimensions, representing the actual distance each cell covers.
- `n_a`: Number of agents.
- `a_t_scan`: Array containing the scan times for each agent.
- `a_t_trav`: Array containing the base travel times for each agent to their current target.
- `a_target`: 3D array specifying the target locations for each agent at each queue position.
- `q`: The current queue position being evaluated.
- `ang_w`, `v_w`, `v_as`: Wind direction (in radians), wind speed, and agent airspeed, respectively.
- `m_t_scan`: Matrix indicating the time at which each cell was last scanned.
- `maxTravelTime`: The maximum allowed travel time, used for normalization and handling cases with undefined travel times.

#### Process

1. **Initialization:**
   - Initialize the response time matrix `m_t_response` with dimensions `(n_x_s, n_y_s, n_a)`, filled with `NaN` values to indicate uncalculated or infeasible travel times.

2. **End Locations Grid:**
   - Generate a grid of all possible end locations `(i, j)` within the environment using `meshgrid`.

3. **Response Time Calculation:**
   - For each agent `a` and each target location `loc_2`:
     - Initialize the base response time with the agent's current travel and scan times.
     - For each queue position up to `q-1`, calculate the travel time from the current target `loc_1` to the next target `loc_next` (or `loc_2` for the last queue position) using `calc_t_trav`.
     - Sum these travel times and the scan time for `loc_2` to obtain the total response time for reaching `loc_2`.
     - If the target list is not depleted (i.e., does not contain `NaN`), assign the calculated response time to the corresponding cell in `m_t_response`.

4. **Normalization:**
   - Normalize the response times to a `[0, 1]` range using `maxTravelTime`. 
   - Replace `NaN` values in `m_t_response` with `maxTravelTime` to handle cases with depleted task lists or infeasible travel times due to constraints like wind speed.

#### Output

- `m_t_response`: A 3D matrix representing the normalized response time for each agent (`n_a`) to reach every location (`n_x_s`, `n_y_s`) in the environment. Values are normalized to the `[0, 1]` range, with `1` representing the `maxTravelTime`.

### `calc_t_trav` Function

The `calc_t_trav` function computes the time required to travel between two points in the presence of wind, taking into account the wind's speed and direction, as well as the agent's airspeed.

#### Inputs

- `loc_1`, `loc_2`: The starting (`loc_1`) and ending (`loc_2`) locations, represented as `[x, y]` coordinates.
- `l_x_s`, `l_y_s`: The scaling factors for the x and y dimensions, representing meters per unit distance in the respective directions.
- `ang_w`: The wind direction angle in radians, measured from the positive x-axis.
- `v_w`: The wind speed in meters per second.
- `v_as`: The airspeed of the agent in meters per second (i.e., its speed relative to the air).

#### Calculations

1. **Distance Calculation (`d`):**
   The Euclidean distance between `loc_1` and `loc_2`, adjusted for scaling factors:
   $$ d = \sqrt{(l_{x_s} \cdot (loc_{2_1} - loc_{1_1}))^2 + (l_{y_s} \cdot (loc_{2_2} - loc_{1_2}))^2} $$

2. **Ground Angle (`a_g`):**
   The angle of the path from `loc_1` to `loc_2` relative to the positive x-axis:
   $$ a_g = \text{atan2}(loc_{2_2} - loc_{1_2}, loc_{2_1} - loc_{1_1}) $$

3. **Wind to Track Angle (`a_wt`):**
   The angle between the wind direction and the ground track direction:
   $$ a_{wt} = a_g - ang_w $$

4. **Wind Correction Angle (`a_wca`):**
   The angle needed to correct for wind drift, calculated using the wind speed, wind to track angle, and airspeed:
   $$ a_{wca} = \text{asin}\left(\frac{v_w \cdot \sin(a_{wt})}{v_{as}}\right) $$
   If the absolute value of the wind ratio (`v_w*sin(a_wt)/v_as`) exceeds 1, travel is deemed impossible due to excessive wind, setting `t_travel` to `Inf`.

5. **Ground Speed (`v_gs`):**
   The actual speed over ground, considering both the airspeed and the wind's effect:
   $$ v_{gs} = v_{as} \cdot \cos(a_{wca}) + v_w \cdot \cos(a_{wt}) $$

6. **Travel Time Calculation (`t_travel`):**
   The time to travel from `loc_1` to `loc_2`, given the ground speed:
   $$ t_{travel} = \frac{d}{v_{gs}} $$
   If `v_gs` is less than or equal to 0, indicating an impossible or undefined travel time due to adverse wind conditions, `t_travel` is set to `Inf`.

#### Output

- `t_travel`: The calculated travel time. If travel is impossible or undefined due to wind conditions, the output is `Inf`.

### `updateFireStatesAndProbabilities` Function

The `updateFireStatesAndProbabilities` function updates the fire states and probabilities within an environmental model over a given time step, considering wind influence and other factors affecting fire spread.

#### Inputs

- `environment_model`: A struct containing the environmental model, including:
  - `m_f`: A matrix representing the fire state of each cell (2 for active, 3 for burning, 4 for burnout).
  - `m_bt`: A matrix representing the burn time for each cell.
  - `t_i`: The ignition time threshold for transitioning from active to burning state.
  - `t_b`: The burnout time threshold for transitioning from burning to burnout state.
  - `n_x_e`, `n_y_e`: The dimensions of the environment.
  - `c_fs_1`, `c_fs_2`: Coefficients for calculating fire spread probability.
  - `m_s`: A matrix representing the slope of each cell.
  - `m_bo`: A matrix representing the building occupancy of each cell.
  - `r_w`: A parameter representing the wind's influence radius.
- `F`: A matrix representing the fire spread probability for each cell.
- `W`: A matrix representing the wind influence on fire spread.
- `dt_e`: The time step for the update.

#### Process

1. **Update Burn Time:**
   - For cells in active (`m_f == 2`) or burning (`m_f == 3`) states, increment the burn time (`m_bt`) by `dt_e`.

2. **Transition Active to Burning:**
   - Cells transition from active to burning state (`m_f` from 2 to 3) if their burn time (`m_bt`) exceeds or equals the ignition time threshold (`t_i`).

3. **Update Fire Spread Probability for Burning Cells:**
   - For each cell in the burning state (`m_f == 3`), calculate the spread probability `p` using `calculateSpreadProbability` based on its burn time (`m_bt`), the ignition time threshold (`t_i`), and the burnout time threshold (`t_b`).
   - Update the fire spread probability matrix `F` for each burning cell using `updateFireSpreadProbability`, considering the calculated probability `p`, wind influence `W`, and other environmental factors.

4. **Transition Burning to Burnout:**
   - Cells transition from burning to burnout state (`m_f` from 3 to 4) if their burn time (`m_bt`) exceeds or equals the burnout time threshold (`t_b`).

#### Outputs

- `environment_model`: The updated environmental model with modified fire states (`m_f`) and burn times (`m_bt`).
- `F`: The updated matrix of fire spread probabilities for each cell.

## Acknowledgments

* Dr Anahita Jamshidnejad, my supervisor.
* Mirko Baglioni, my PhD supervisor.
