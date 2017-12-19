'''
Code is written in Python 2
Install the list of packages in the Pipfile using PyEnv


By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Dec 2017
Last update: Dec/18/2017
'''
import win32com.client as com
import sys
import os

if __name__ == "__main__":
    print('Python Path: ', sys.executable)
    print('Python Version: ', sys.version)

    Vissim = com.Dispatch("Vissim.Vissim-64.900") # Vissim 9 - 64 bit