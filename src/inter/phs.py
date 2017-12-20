'''
Creates library of all phases


By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Dec 2017
Last update: Dec/03/2017
'''

from itertools import combinations
import numpy as np
import os


def phenum(l, lli, name):
    '''

    :param l: number of lanes
    :param lli: link lane incidence matrix
    :return:
    '''

    all_combs = []
    phlen = 1

    # of all generated subset of lanes
    # of self conflicting  phases
    # of removed phases by inclusion rule
    nall, ncnf, ninc = 0, 0, 0

    while True:
        new_comb = []
        for ph in combinations(range(0, l), phlen):
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

    write2txt(all_combs, l, name)


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


def write2txt(all_combs, l, name):
    filepath = os.path.join('data/' + name, 'PLI.txt')
    with open(filepath, "w") as text_file:
        for ph in range(0, len(all_combs)):
            lane = 0
            if lane in all_combs[ph]:
                text_file.writelines("1")
            else:
                text_file.writelines("0")
            lane = 1
            while lane < l:
                if lane in all_combs[ph]:
                    text_file.writelines(",1")
                else:
                    text_file.writelines(",0")
                lane += 1
            text_file.writelines("\n")
