# Recommendations

This document details possible future additions to the project that will not be a part of the initial release.
Feel free to take any of these additions as inspiration for contributing to the project.

* Modelling
 * Agent modelling - The agent model used in this project is an abstract discrete 2D model with simple dynamics for movement and scanning.
A more advanced model could choose a type of agent to model, for instance quadcopter uavs, and include 3D dynamics modelling (aerodynamics, structure, etc.), sensor modelling (camera, infrared, cellular network communications, etc), and electronics modelling (battery, etc).
 * Environment modelling - The environment model used in the simulation is a simple discrete 2D top-down model of a disaster environment and includes static models for wind and building coverage, and a cellular automata-based dynamic model, for fire.
Further work could develop a 3D environment model based on LIDAR datasets of disaster environments, 3D dynamic modelling of wind, and physics-based 3D modelling of fire.
Additional models could also be implemented in the simulation if they could have relevance to the operation of the system, such as smoke, victim location, search team, charging stations, building damage, and so on.
* Controller Design
 * Path planning controller - If 3D simulation models are implemented, the path planning controller should also operate with 3D waypoints.
Further changes could include obstacle avoidance implementation for navigating a 3D environment and discretisation of the environment into polygonal search cells, instead of dividing it into a grid, according to the fastest search patterns - for example, identifying individual buildings as separate search cells, as it would be most efficient to scan one building all at the same time.
The path planning controller could also be a focus of computational efficiency improvements.
For example, currently the path planning controller will calculate the attraction of all unscanned cells at each path planning time step, whereas it could potentially analyse a constant \textit{n} closest cells with negligible drop in performance.
 * MPC controller - Studies focusing on the MPC design could look at the optimal choice of objective function for the MPC module, the optimal solver configuration for the optimisation, the optimal setup of the MPC (prediction horizon, etc), and moving the MPC model to a probability-based model, as in a real application the system would not have access to perfect information as is assumed in this project.
The MPC solver could also implement multi-objective optimisation to consider other important objectives of the agents, such as battery life limitations and recharging, etc.
 * FIS Design - Further studies could focus on the design of the FIS by analysing the optimal combinations of inputs, number and shape of membership functions, number of output surfaces, and design of the rule base.
Potential new inputs for the FIS could include different methods to categorise risk to victims due to dynamic environment variables, such as the \textit{downwind time} used in this project.
This line of research should also focus on determining a balance between the complexity of the FIS and the computational power available for the FIS in a real-world application.
* Validation
 * Scaled Experiment - After satisfactory simulation results with appropriately accurate modelling, a scaled experiment of the system should be conducted in a controlled environment to determine any inconsistencies between simulation and reality.
 * Scalability - The scalability of the system should be analysed through simulation or experimentation to determine any limitations on the operation of the system, as a real-life application may require a large number of agents depending upon the scale of the disaster environment.
 * Robustness - The system should be robust to disruptions and disturbances in the system, such as losing agents to environmental hazards, erroneous measurements in the system, and so on.
 * System Design
 
* Incorporate additional a-priori data for simulation
  * Earthquake data - https://www.gislounge.com/haiti-maps-and-gis-data-resources
  * Population distribution
  * etc...
* Implement error checking across simulation
* Selection criteria for path-planning algorithm (e.g. 100 nearest cells)
