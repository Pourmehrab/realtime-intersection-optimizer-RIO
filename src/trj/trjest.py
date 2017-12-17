'''
Code is written in Python 3
Install the list of packages in the Pipfile using PyEnv


By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Dec 2017
Last update: Dec/14/2017
'''
import numpy as np

from src.trj import MahmoudTrj


class MahmoudCNVE(MahmoudTrj):
    '''
    Goal: Estimating movement of a conventional vehicle
    '''
    def __init__(self,lead_veh, fol_veh, gs=0, gt=86400, vmax=15, vcont=10):

        super().__init__(lead_veh, fol_veh, gs, gt, vmax, vcont)