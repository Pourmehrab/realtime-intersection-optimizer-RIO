'''
This class is meant to slove the min cost flow problem that is set up for phase selection

Code is written in Python 3
Install the list of packages in the Pipfile using PyEnv


By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Dec 2017
Last update: Dec/15/2017
'''

from ortools.graph import pywrapgraph


class SigNet:
    '''
    NODES:
    Head of phase-selection arcs: from 0 to |p|-1
    Head of sink arcs: from |p| to 2|p|-1
    Head of lane-assignment arcs: from 2|p|+1 to 2|p|+|L|

    ARCS:
    Phase-selection arcs are from 0 to |p|-1
        cost 1 unit
    Sink arcs are from |p| to 2|p|-1
        cost 0 unit
    Lane-assignment arcs are from 2|p| to len(A)
        cost 0 unit
    (Arcs have no capacity let's say M)


    '''
    M = 999  # todo hash the node number based on p and l

    def __init__(self, num_lanes, ppi):
        num_ph = ppi.shape[0]
        self.num_lanes = num_lanes

        # to find out relevant nodes

        # add phase selection arcs
        self.start_nodes = [p for p in range(num_ph)]
        self.end_nodes = [num_ph + p for p in range(num_ph)]
        self.unit_costs = [1 for p in range(num_ph)]

        # add sink arcs
        self.start_nodes += [num_ph + p for p in range(num_ph)]
        self.end_nodes += [2 * num_ph for p in range(num_ph)]
        self.unit_costs += [0 for p in range(num_ph)]
        self.demand_nodes = [num_ph + p for p in range(num_ph)]

        # add lane to phase arcs
        index = len(self.start_nodes) - 1
        for p in range(num_ph):
            for l in range(self.num_lanes):
                if ppi[p, l]:
                    index += 1
                    self.start_nodes.append(2 * num_ph + 1 + l)
                    self.end_nodes.append(p)
                    self.unit_costs.append(0)

        self.capacities = [self.M for n in range(len(self.start_nodes))]

        self.supplies = [0 for n in range(2 * num_ph + self.num_lanes + 1)]

    def set_dem(self, lanes_demand):
        total_dem = 0
        for l, d in enumerate(lanes_demand):
            total_dem -= d
            self.supplies[l - self.num_lanes] = d

        self.supplies[-1 * self.num_lanes - 1] = total_dem

    def solve(self):
        min_cost_flow = pywrapgraph.SimpleMinCostFlow()

        # Add each arc.
        for i in range(0, len(self.start_nodes)):
            min_cost_flow.AddArcWithCapacityAndUnitCost(self.start_nodes[i], self.end_nodes[i],
                                                        self.capacities[i], self.unit_costs[i])

        # Add node supplies.
        for i in range(0, len(self.supplies)):
            min_cost_flow.SetNodeSupply(i, self.supplies[i])

        # Find the minimum cost flow between node 0 and node 4.
        if min_cost_flow.Solve() == min_cost_flow.OPTIMAL:
            print('Solution to MCF found.')
            # print('Minimum cost:', min_cost_flow.OptimalCost())
            # print('')
            # print('  Arc    Flow / Capacity  Cost')
            # for i in range(min_cost_flow.NumArcs()):
            #     cost = min_cost_flow.Flow(i) * min_cost_flow.UnitCost(i)
            #     print('%1s -> %1s   %3s  / %3s       %3s' % (
            #         min_cost_flow.Tail(i),
            #         min_cost_flow.Head(i),
            #         min_cost_flow.Flow(i),
            #         min_cost_flow.Capacity(i),
            #         cost))
        else:
            raise Exception('There was an issue with the min cost flow input.')
