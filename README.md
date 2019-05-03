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

When `RIO.yml` is updated, you can update your already created RIO environment by `conda env update -f RIO.yml`.

## Documentation

TODO: Instructions here. 

### Network field connection instructions

The RSU uses a wired Link-Local connection. Set the local ip address to be something like 169.254.X.X. By default, it is set to 169.154.117.41.
The signal controller and other sensors are connected on ip addresses starting with 192.168.X.X. By default, the signal controller's IP
is set to 192.168.91.71.

## Tests

Run all unit tests with `pytest -v test`.

## Visualizations

Use the argument `--show-viz=True` during `sim` mode to have the system plot the current state of the traffic after every iteration. Use the argument `--save-viz=True` to save the plots to a file. Plots are automatically saved in `log/LOG_DIRNAME/imgs/` as .pngs.

To create a .gif out of the images, do `python src/visualizer.py --img-dir log/LOG_DIRNAME/imgs`

which produces `out.gif` in the image directory.

### Example for RTS intersection

![RTS](examples/out.gif)


