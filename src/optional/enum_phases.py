####################################
# File name: enum_phases.py                #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Feb/16/2018       #
####################################

from itertools import combinations
import numpy as np
import os


def phenum(num_lanes, lli_dict, name):
    '''

    :param num_lanes: number of lanes
    :param lli: link lane incidence matrix
    :return:
    '''

    lli = np.zeros((num_lanes, num_lanes))

    for l, conf in lli_dict.items():
        for j in conf:
            lli[l, j] = 1

    all_combs = []
    phlen = 1

    # of all generated subset of lanes
    # of self conflicting  phases
    # of removed phases by inclusion rule
    nall, ncnf, ninc = 0, 0, 0

    while True:
        new_comb = []
        for ph in combinations(range(0, num_lanes), phlen):
            new_comb.append(ph)
        nall += len(new_comb)

        non_conf_combs = chkconflict(new_comb, lli)

        if len(non_conf_combs) == 0:  # all conflicting
            ncnf += len(new_comb)
            break
        else:  # some conflicting
            ncnf += len(new_comb) - len(non_conf_combs)

            if len(all_combs) > 0:
                all_combs, ninc = chkinc(all_combs, non_conf_combs, ninc)
            else:
                all_combs += non_conf_combs

        phlen += 1

    print('Phase Generator Report enumerated : {}, conflicting: {}, inclusion: {}, remaining: {}\n'.format(
        nall, ncnf, ninc, len(all_combs)))

    write2txt(all_combs, name)
    print('phases are written to file in log directory. Add them to data.py and rerun')


def chkconflict(new_comb, lli):
    '''

    :param new_comb:
    :param lli:
    :return: list of non conflicting
    '''
    refined_combs, n = [], 0

    for comb in new_comb:
        total = 0
        for jj in comb:
            total += np.sum(np.sum(lli[jj, comb]))

        if total == 0:
            refined_combs.append(comb)

    return refined_combs


def chkinc(all_combs, non_conf_combs, ninc):
    indx = 0
    while indx < len(all_combs):
        if len(all_combs) == 0:
            print('error')
        t1 = all_combs[indx]
        for t2 in non_conf_combs:
            flag = True
            if set(t1).issubset(t2):
                ninc += 1
                del all_combs[indx]
                flag = False
                break
        if flag:
            indx += 1
        elif len(all_combs) == 0:
            break
    all_combs += non_conf_combs
    return all_combs, ninc


def write2txt(all_combs, name):
    filepath = os.path.join('log/', name + '_pli_dictionary.txt')
    with open(filepath, "w") as text_file:
        for ph in range(len(all_combs)):
            text_file.write('{:d} : {{'.format(ph + 1))
            for lane in all_combs[ph]:
                text_file.write('{:d}, '.format(lane + 1))
            text_file.write('},\n')
