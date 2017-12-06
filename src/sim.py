'''
By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Last update: November/2017
'''

from src.inpt import read_prms


class Simulator:
    def __init__(self, int_name):
        d = {"simRep": "simRep", "simDur": "simDur", "warmup": "warmup", "simRes": "simRes", "timeRes": "timeRes"}
        # time : sec
        read_prms(self, int_name, 'sim', d)
