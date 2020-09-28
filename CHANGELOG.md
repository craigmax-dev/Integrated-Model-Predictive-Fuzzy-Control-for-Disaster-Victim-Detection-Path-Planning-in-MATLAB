# CHANGELOG

This document contains an archive of issues completed before this project was moved to GitHub.

## main.m

- Changed FIS output surface to decrease with time
- Implemented modular code design
- Implemented new code styling standard across project
- Added fire component to victim risk calculation
- - constant weighting added while fire is active
- Designed input for FIS to characterise fire spread
- - downwindfire proximity - linear function of distance and windspeed related to nearest active fire
- Implemented fire spread probability matrix based on wind speed.
- Fixed bug where fuzzy rules were not being fired.
- Plot changes:
- - animated plots
- - added lat/lon to axes
- - multiplot
- Implemented separate timesteps for all components of the simulation - MPC,
- FIS, Control, Dynamics.
- Implemented UAV actions independent of others.
- Implemented model for scan cell priority - constant function of building area
- coverage
- Implemented function to save figures and simulation data to specified folder.
- Implemented simulation progress report
- 30/06/2020 - Get simulation working for UAV's without MPC part
- 30/06/2020 - Fixed objective function evaluation
- 30/06/2020 - UAV_target initialised at start of simulation
- 01/07/2020 - Changed order of script execution in main
- 01/07/2020 - Reorganised counters
- 02/07/2020 - Added MPC model to simulation
- 03/07/2020 - Changed fmincon to fminsearch
- 04/07/2020 - Added plotting of optimisation
- 17/07/2020 - Scaling problem investigation
- - Objective function parts
- - Fuzzy inputs and associated output
- 17/07/2020 - Changed exportData function - save all workspace variables
- 20/07/2020 - Implemented test logic
- 30/07/2020 - Removed m_att to simplify code
- 30/07/2020 - add time measurement to all modules of simulation
- 30/07/2020 - created time measurement function
- 30/07/2020 - replaced nested sum with sum(a,'all') across all functions
- 01/08/2020 - bugfix - m_dw not coarsened to search map size
- 04/08/2020 - added error checking script in preamble
- 04/08/2020 - added simulation time estimation
- 04/08/2020 - added choice of number of datapoints over simulation
- 08/08/2020 - bugfix - fixed error with wrong dimention m_att being passed to
- taskAssignment function
- 10/08/2020 - fixed m_dw calculation
- 10/08/2020 - separated coarsen and coarsenFactor calculation
- 12/08/2020 - changed recording of m_scan and m_scan_hist
- 14/08/2020 - fixed saving simulation results
- 14/08/2020 - added plotting of testSensitivity results
- 15/08/2020 - added patternsearch and particleswarm options for MPC solver
- 17/08/2020 - cleaned up export folder generation and add sim name
- 18/08/2020 - created script for comparison of results from multiple
- simulations
- 14/09/2020 - changed notation to match paper

## coarsen.m

- 22/06/2020 - improved readability
- 22/06/2020 - removed errata with idea of randomising building flammability
- 22/07/2020 - Implemented option for coarsening using coarsening factor
- 22/07/2020 - Added description
- 10/08/2020 - Separated coarsening and coarsen factor calculation

## pathPlanner.m

- 22/06/2020 - Removed fireModel evaluation from script
- 22/06/2020 - Removed UAV_occ flag - instead "NaN" can be used inside UAV tasks
- 22/06/2020 - Fixed confusing indexing in travel time calculation
- 22/06/2020 - Fixed travel time calculation
- 22/06/2020 - Expanded travel time calculation for any value of l_queue
- 22/06/2020 - Travel times saved after calculation
- 29/06/2020 - Bugfix - problems with path planning when entire environment
- scanned.
- 01/07/2020 - Added empty attraction map consideration to conflict resolution
- script
- 27/07/2020 - Normalisation of time input - same for all UAVs but can change in
- queue
- 28/07/2020 - Simplified skip criteria
- 30/07/2020 - bugfix - m_scan_schedule and t_nextcell_mat handling
- 30/07/2020 - bugfix - schedule conflict execution condition
- 30/07/2020 - syntax fix - replace nested sum function with sum(mat,'all')
- 30/07/2020 - bugfix - task reassignment algorithm using wrong indices in if
- statement
- 30/07/2020 - syntax fix - use max(mat,[],'all') instead of nested max
- 30/07/2020 - created task assignment function
- 30/07/2020 - created attraction function
- 30/07/2020 - created timeToNextCell function

## agentModel.m

- 22/06/2020 - reworked agent task assignment
- 22/06/2020 - corrected agent distance calculations
- 22/06/2020 - improved readability

## attcalc.m

- 01/08/2020 - changed to use arrayfun()

## travelTime.m

- 08/08/2020 - bugfix - incorrect brackets causing incorrect distance
- calculation

## timeToNextCell.m

- 08/08/2020 - bugfix - loc_2 assignment

## taskAssignment.m

- 01/08/2020 - removed use of n_x_search and n_y_search

## genPlots.m

- 22/06/2020 - colormaps defined in script
- 29/06/2020 - added UAV path plotting
- 29/06/2020 - change obj_hist plot from scatter to plot
- 10/08/2020 - bugfix - incorrect time in environment variable plots
- 10/08/2020 - improvement - removed unneccesary code
- 17/08/2020 - added FIS parameter history plotting - scatter and surface

## objEval.m

- 22/06/2020 - ignore scanned cells in calculation
- 22/06/2020 - fixed bug with non-integers being used for matrix searches
- 22/06/2020 - fixed bug with converting m_search to same dimension as m_scan
- 29/06/2020 - fixed bug with calculation of m_scan_env_inv
- 17/07/2020 - Added obj_fun_scaling
