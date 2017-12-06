'''
By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Last update: November/2017
'''
import os


def read_prms(self, int_name, prms_id, d):
    filepath = os.path.join('data/' + int_name, prms_id + 'prms.txt')
    if os.path.exists(filepath):
        file = open(filepath)
        for line in file:
            name, value = line.split(":")
            value = value.strip()
            if " " in value:
                value = map(float, value.split())
            else:
                value = float(value)
            setattr(self, d[name], value)
        return None
    else:
        raise Exception(filepath + ' was not found.')
