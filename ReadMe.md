# General Description
SI unit is used.

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
* `CM.txt`: intersection conflict matrix (LLI)
* `intprms.txt`: intersection parameters
* `MS.txt`: movement share
* `PPI.txt`: phase-lane incidence matrix (code generates this if not there)
* `simprms.txt`: simulation parameters

# Files
## `runOpt.py`:
This files is the main file should be run. It moderates all else.


## Useful Links
* Markdown hints [here](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet)
* Paper is on overleaf [here](https://www.overleaf.com/9570639sgrcxsbwcxxm)





