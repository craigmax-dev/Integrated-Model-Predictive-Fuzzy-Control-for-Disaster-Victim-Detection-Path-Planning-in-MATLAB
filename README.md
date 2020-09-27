# Integrated Model Predictive Fuzzy Control for Disaster Victim Detection Path Planning in MATLAB

This is my thesis project for my MSc in Control and Simulation, Aerospace Engineering at the Delft University of Technology. The code is currently in pre-alpha.

This project consists of a simulation of a search-and-rescue environment for discrete path-planning of agents using a Fuzzy Inference System (FIS)-based controller and a Model Predictive Control (MPC)-based controller to optimise FIS parameters. The environment model consists of static states (building coverage and wind) and dynamic states (fire and agents). The inputs used for the FIS are distance, priority, and downwind time; each of which are modelled in the simulation. The output of the FIS is attraction which is used by the path-planner to decide on waypoint locations for the UAVs. The MPC uses an objective function which is a combination of the accumulative priority of unscanned cells and an additional priority due to cells which are on fire. The MPC uses the patternsearch solver for the nonlinear time-limited optimisation of the FIS parameters.

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

Default tests for the project are not ready yet and will be deployed with the final version of the code.

## Contributing

This is a thesis project as part of my final project for my MSc in Control and Simulation, Aerospace Engineering at the Delft University of Technology. Contributions will be welcome after the final version of the code for the thesis is produced. There are many options to continue building on and improving the code. 
Please read [CONTRIBUTING.md](https://gist.github.com/craigmax-dev/contributing) for details on our code of conduct, and the process for submitting pull requests to us. For inspiration or ideas of how to contribute to the project, please read [RECOMMENDATIONS.md](RECOMMENDATIONS.md)

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags). 

## Authors

* **Craig Maxwell** - *Initial work* - [craigmax-dev](https://github.com/craigmax-dev)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Anahita Jamshidnejad, my supervisor.
