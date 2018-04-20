[TOC]

# General Description
* SI units(speed in m/s, length in m, time in sec, acceleration in m/s2)
* Python 3.5.4 used.
* run `python main <intersection_name> <optimization_algo>`
*   intersection name could be `13th16th` or `reserv`
*   optimization algorithm could be `GA, pretimed` or `MCF`

Coded by: Mahmoud Pourmehrab (mpourmehrab@ufl.edu)


## Package Dependencies
To install:
* Do `pip3 install -r requirements.txt`

[//]: # (To create:
* Do `pip3 install pipreqs` or `conda install pipreqs` 
* `requirements.txt` is created using `pipreqs /path/to/project`.
Warp the address with "".)

# Input Data
Directory `\data\` includes:
* `<intersection name>.csv`: includes scenarios to be tested. Filename should match the intersection name.

# Output Data
Directory `\log\` includes:
* `<intersection name>_vehicle_level, _trj_point_level.csv`: includes input csv plus the `departure time` and `elapsed time` columns.
# Notes
* `signal.solve()` extends by each optimization method, i.e, `GA` or `MCF`.
* simulation resolution is set as a class variable `Simulation.STEP` in seconds
* 

# LP Solver:

* Packages can solve LP: `cvxopt, Scipy.minimize, docplex`
* Requires Microsoft Visual C++ Build Tools [here](http://landinghub.visualstudio.com/visual-cpp-build-tools)
* IBM ILOG CPLEX Optimization Studio Version 12.8.0 (students can request free license)
	* Do `pip install docplex`. Find more here:
		*  [Documentation](http://ibmdecisionoptimization.github.io/docplex-doc/)
		* [Examples](https://github.com/IBMDecisionOptimization/docplex-examples)

* Microsoft Visual C++ 2015 Redistributable Package (x64) [here](https://www.microsoft.com/en-US/download/details.aspx?id=53587)

## PyCharm
* Some useful plugins are `Markdown Support`, `CSV Plugin`, `RainGlow`, and `CodeGlance`.
