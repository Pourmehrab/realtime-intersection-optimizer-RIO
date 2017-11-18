'''
By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Last update: November/2017
'''


def readprms(self, intName, fileID, d):
    filepath = "./data/" + intName + "/" + fileID + "prms.txt"
    FILE = open(filepath)
    for line in FILE:
        name, value = line.split(":")
        value = value.strip()
        if " " in value:
            value = map(float, value.split())
        else:
            value = float(value)
        setattr(self, d[name], value)
