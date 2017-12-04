'''
Creates library of all phases


By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Dec 2017
Last update: Dec/03/2017
'''

from itertools import combinations
import numpy as np


def phenum(l, LLI):
    '''

    :param l: number of lanes
    :param LLI: link lane incidence matrix
    :return:
    '''

    allCombs = []
    phlen = 1

    # of all generated subset of lanes
    # of self conflicting  phases
    # of removed phases by inclusion rule
    nall, ncnf, ninc = 0, 0, 0

    while True:
        newComb = []
        for ph in combinations(range(0, l), phlen):
            newComb.append(ph)
        nall += newComb.__len__()

        nonConfCombs = chkconflict(newComb, LLI)

        if nonConfCombs.__len__() == 0:  # all conflicting
            ncnf += newComb.__len__()
            break
        else:  # some conflicting
            ncnf += newComb.__len__() - nonConfCombs.__len__()

            if allCombs.__len__() > 0:
                allCombs, ninc = chkinc(allCombs, nonConfCombs, ninc)
            else:
                allCombs += nonConfCombs

        phlen += 1

    print('Phase Generator Report enumerated : {}, conflicting: {}, inclusion: {}, remaining: {}\n'.format(
        nall, ncnf, ninc, allCombs.__len__()))
    return allCombs


def chkconflict(newComb, LLI):
    '''

    :param newComb:
    :param LLI:
    :return: list of non conflicting
    '''
    list, n = [], 0

    for comb in newComb:
        sum = 0
        for jj in comb:
            sum += np.sum(np.sum(LLI[jj, comb]))

        if sum == 0:
            list.append(comb)

    return list


def chkinc(allCombs, nonConfCombs, ninc):
    indx = 0
    while indx < allCombs.__len__():
        if allCombs.__len__() == 0:
            print('error')
        t1 = allCombs[indx]
        for t2 in nonConfCombs:
            flag = True
            if set(t1).issubset(t2):
                ninc += 1
                del allCombs[indx]
                flag = False
                break
        if flag:
            indx += 1
        elif allCombs.__len__() == 0:
            break
    allCombs += nonConfCombs
    return allCombs, ninc

