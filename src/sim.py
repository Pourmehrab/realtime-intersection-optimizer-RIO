'''
By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Last update: November/2017
'''

from src.inpt import readprms


class Simulator:
    def __init__(self, intName):
        d = {"simRep": "simRep", "simDur": "simDur", "warmup": "warmup", "simRes": "simRes", "timeRes": "timeRes"}
        # time : sec
        readprms(self, intName, 'sim', d)
