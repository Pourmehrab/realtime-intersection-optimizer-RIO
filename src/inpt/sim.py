'''
By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        November 2017
Last update: Dec/08/2017
'''
from src.inpt.inpt import read_prms


class Simulator:
    def __init__(self, int_name):
        d = {'simulation_repetition':'_sim_rep',"simulation_duration": "_sim_dur",
             "warmup_duration": "_warmup", "optimization_frequency": "_freq"}
        # time : sec
        read_prms(self, int_name, 'sim', d)
