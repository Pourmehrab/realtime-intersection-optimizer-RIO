'''
Goal: Make network of min cost flow for publication

Don't forget the header for loading packages:

\\usepackage{tikz}
\\usetikzlibrary{arrows,decorations.pathmorphing,backgrounds,positioning,fit,petri}


By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Dec 2017
Last update: Dec/17/2017
'''

import os
import numpy as np

class TikzDirectedGraph:
    H = 3  # height of layers
    W = 1  # width of steps

    def __init__(self, inter_name, num_lanes, ppi):
        self.filepath = os.path.join('log/' + inter_name, 'tikz_graph.txt')
        if os.path.exists(self.filepath):
            self._f = open(self.filepath, 'w')
        else:
            self._f = open(self.filepath, 'x')

        self.num_lanes = num_lanes
        self.ppi = ppi

        self._make_header()
        self._add_nodes()
        self._add_arcs()

        self._closefile()

    def _make_header(self):
        string = '\\begin{tikzpicture}\n\n'
        self._f.write(string)

    def _add_nodes(self):
        self._f.write('%NODES\n')

        x_left = self.comp_x_left(self.num_lanes)
        x = x_left
        y = 3 * self.H
        for l in range(self.num_lanes):
            lane = l + 1
            self._f.write(
                '\\node (l{:d}) at ( {:2.2f},{:2.2f}) [circle,draw,inner sep=0pt,minimum size=6mm] {{$l_{{{:d}}}$}};\n'.format(
                    lane, x, y, lane))
            self._f.write('\\node [black,above] at (l{:d}.north) {{{{\\tiny $(+d_{{{:d}}})$}}}};\n'.format(lane, lane))
            x += self.W


        x_left = self.comp_x_left(len(self.ppi))
        x = x_left
        y = 2 * self.H
        for ph in range(len(self.ppi)):
            phase = ph + 1
            self._f.write(
                '\\node (p{:d}) at ( {:2.2f},{:2.2f}) [circle,draw,inner sep=0pt,minimum size=6mm] {{$p_{{{:d}}}$}};\n'.format(
                    phase, x, y, phase))
            self._f.write(
                '\\node (pp{:d}) at ( {:2.2f},{:2.2f}) [circle,draw,inner sep=0pt,minimum size=6mm] {{$p_{{{:d}}}^\prime$}};\n'.format(
                    phase, x, y - self.H, phase))
            x += self.W
        self._f.write(
            '\\node (sink) at ( {:2.2f},{:2.2f}) [circle,draw,inner sep=1.5pt,minimum size=6mm] {{$sink$}};\n'.format(
                0, 0))
        self._f.write('\\node [black,below] at (sink.south) {{$(-\\sum_{{l=1}}^{{16}} d_{{l}})$}};\n')

    def comp_x_left(self, n):
        if n % 2 == 0:
            return -1 * self.W * self.num_lanes // 2
        else:
            return -1 * self.W * self.num_lanes // 2 + self.W / 2

    def _add_arcs(self):
        # \draw[->, line width = 1.8pt] FYI
        self._f.write('%ARCS\n')
        # add lane to phase arcs
        for p in range(len(self.ppi)):
            for l in range(self.num_lanes):
                if self.ppi[p, l]:
                    self._f.write('\\draw[->,> = latex,semithick] (l{:d}) to (p{:d});\n'.format(l + 1, p + 1))

        max_ph_size = max(np.sum(self.ppi, 1))
        for p in range(len(self.ppi)):
            self._f.write(
                '\\draw[->,> = latex,semithick,bend right=20] (p{:d}) to node [below,align=center,rotate=90,midway] {{{{\\tiny ${:d}/\\infty$}}}} (pp{:d});\n'.format(
                    p + 1, int(1 + max_ph_size - sum(self.ppi[p]) + 1), p + 1))

            self._f.write(
                '\\draw[->,> = latex,semithick,bend left=20] (p{:d}) to node [above,align=center,rotate=90,midway] {{{{\\tiny $1/1$}}}} (pp{:d});\n'.format(
                    p + 1, p + 1))
            self._f.write('\\draw[->,> = latex,semithick] (pp{:d}) to (sink);\n'.format(p + 1, p + 1))

    def _closefile(self):
        self._f.write('\end{tikzpicture}')
        self._f.close()
        print('done')
