[TOC]

# General Description
* SI units(speed in m/s, length in m, time in sec, acceleration in m/s2)
* Python 3.5.4 used.

Coded by: Mahmoud Pourmehrab 

## PyCharm Set up
* To use Markdown in PyCharm install "Markdown Support" under Plugin menu

## Package Dependencies
To create:
* Do `pip3 install pipreqs` or `conda install pipreqs` 
* Requrement.txt is created using `pipreqs /path/to/project`
(Warp the address with "")

To install:
* Do `pip3 install -r requirements.txt`

# Input Data
Directory `\data\<intersection name>` includes:
* `<intersection name>.csv`: includes scenarios to be tested.

# Important Files
## `main.py`:
This files is the main file should be run. It controls all else.



## LP Solver for Traj Optimization:

* Packages can solve LP: `cvxopt, Scipy.minimize, docplex`
* Requires Microsoft Visual C++ Build Tools [here](http://landinghub.visualstudio.com/visual-cpp-build-tools)
* IBM ILOG CPLEX Optimization Studio Version 12.8.0
	* Do `pip install docplex`. Find more here:
		*  [Documentation](http://ibmdecisionoptimization.github.io/docplex-doc/)
		* [Examples](https://github.com/IBMDecisionOptimization/docplex-examples)

* Microsoft Visual C++ 2015 Redistributable Package (x64) [here](https://www.microsoft.com/en-US/download/details.aspx?id=53587)

## Useful Links
* Markdown hints [here](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet)
* Paper is on overleaf [here](https://www.overleaf.com/9570639sgrcxsbwcxxm)
