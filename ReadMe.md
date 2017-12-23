# General Description
SI unit, Python 3.6.3 used.

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
Directory `\data` includes folders named after each intersection of interest. Includes:
* `CM.txt`: intersection conflict matrix (LLI: 1 if i,j are conflicting) [may create it by running `test.py>` for 13th and 16th]
* `intprms.txt`: intersection parameters
* `MS.txt`: movement share
* `PLI.txt`: phase-lane incidence matrix (code generates this if not there)
* `simprms.txt`: simulation parameters

# Files
## `runOpt.py`:
This files is the main file should be run. It moderates all else.


## Useful Links
* Markdown hints [here](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet)
* Paper is on overleaf [here](https://www.overleaf.com/9570639sgrcxsbwcxxm)

## LP Solver for Traj Optimization:

* Requires Microsoft Visual C++ Build Tools [here](http://landinghub.visualstudio.com/visual-cpp-build-tools)
* IBM ILOG CPLEX Optimization Studio Version 12.8.0
* Microsoft Visual C++ 2015 Redistributable Package (x64) [here](https://www.microsoft.com/en-US/download/details.aspx?id=53587)

* package name `cvxopt`
