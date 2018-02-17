# key is a lane : vals are lanes that are in conflict with key
conf_dict = {1: [7],
             2: [7, 8, 12, 16, 15, 14, 13],
             3: [7, 8, 12, 16, 15, 14, 9],
             4: [7, 16, 8, 15, 9, 11, 10],
             5: [16, 7, 15, 8, 11, 9, 10, 14],
             6: [10],
             7: [10, 15, 11, 16, 5, 4, 3, 2, 1],
             8: [10, 11, 15, 5, 4, 163, 2, 12],
             9: [5, 10, 4, 11, 14, 12, 13, 3],
             10: [13, 14, 4, 5, 9, 8, 7, 6, 15],
             11: [13, 14, 9, 4, 5, 8, 7, 15, 16],
             12: [13, 14, 9, 3, 15, 16, 2, 8],
             13: [2, 3, 9, 12, 11, 10, 4],
             14: [2, 3, 12, 9, 11, 4, 10, 5],
             15: [2, 3, 12, 4, 5, 8, 7, 11, 6],
             16: [12, 2, 3, 8, 4, 7, 5, 11]}

zero_based_conf_dict = {i: set([]) for i in range(16)}

for l1 in range(16):
    for l2 in conf_dict[l1 + 1]:
        zero_based_conf_dict[l1].add(l2 - 1)


# DP

def hash2bin(s):
    '''
    hashes set s to binary
    :param s: set with subset of {0,...,n}
    :return: string of binary
    '''
    num = 0
    for i in s:
        num += 10 ** (i)
    return num


def set_union(phi, LLI):
    X = set([])
    for l in phi:
        X = X | LLI[l]

    return X


L = set([i for i in range(16)])
big_phi = set([i for i in range(16)])


def chk_subset(bin_phi, big_phi):
    for p in big_phi:
        if bin_phi < p:
            n = p - bin_phi
            if '9' not in str(n):
                return True
    return False


def inc_wise_phase(
        phi,
        big_phi,
        big_phi_complement,
        L, LLI):
    '''

    :param big_phi: a set of subsets of L (like memo in DP: we start by empty set)
    :param L: set of all lanes
    :param LLI: Lane-Lane Incidence in dictionary starts from 0
    :return:
    '''
    X = set_union(phi, LLI)  # all lanes in conflict with current phase
    feasible_L = L - X - phi  # feasible lanes to add to current phase
    if len(feasible_L) == 0:
        return phi, big_phi, big_phi_complement
    else:
        for n, l in enumerate(feasible_L, start=1):  # try all feasible lane additions
            phi_new = phi | {l}  # add lane to current phase
            phi_new_bin = hash2bin(phi_new)
            if phi_new_bin in big_phi | big_phi_complement:  # true if this case has occurred before
                # return phi, big_phi, big_phi_complement
                continue
            elif chk_subset(phi_new_bin, big_phi):
                big_phi_complement.add(phi_new_bin)
                continue
            else:  # new phase dominates the one before lane addition
                phi_bin = hash2bin(phi)
                big_phi_complement.add(phi_bin)
                big_phi.discard(phi_bin)
                big_phi.add(phi_new_bin)

                inc_wise_phase(phi_new, big_phi, big_phi_complement, L, LLI)

        return phi, big_phi, big_phi_complement


def dehash(phi):
    for p in phi:
        print('\n')
        s = str(p)
        for n, c in enumerate(reversed(s), start=1):
            if c == '1':
                print(', {:d}'.format(n), end='')


phi, big_phi, big_phi_complement = inc_wise_phase(set(), set(), set(), L, zero_based_conf_dict)
dehash(big_phi)
print(phi, big_phi, big_phi_complement)
