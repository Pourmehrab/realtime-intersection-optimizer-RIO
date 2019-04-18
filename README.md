* Visit [here](https://pourmehrab.github.io/RIO/) for the documentation
* Visit [here](http://avian.essie.ufl.edu/) for project website
* Questions/Comments:

Mahmoud Pourmehrab
[pourmehrab@gmail.com](mailto:pourmehrab@gmail.com)

Ash Omidvar
[aschkan@ufl.edu](mailto:aschkan@ufl.edu)

Patrick Emami
[pemami@ufl.edu](mailto:pemami@ufl.edu)

## Installation

### Anaconda

1. If needed, download and setup [Anaconda](https://docs.anaconda.com/anaconda/install/)
2. In the base directory of this repository, do `conda env create -f RIO.yml`

This will create a conda env that can be activated with `source activate RIO`. All dependencies needed to run the code will be installed.

## Documentation

TODO: Instructions here. 

### Network instructions

The RSU uses a wired Link-Local connection. Set the local ip address to be something like 169.254.X.X. By default, it is set to 169.154.117.41.
The signal controller and other sensors are connected on ip addresses starting with 192.168.X.X. By default, the signal controller's IP
is set to 192.168.91.71.

## Tests

Run all unit tests with `pytest -v test`.
