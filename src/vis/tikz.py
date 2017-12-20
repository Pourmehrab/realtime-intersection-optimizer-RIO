'''
Goal: Make network of min cost flow for publication

Don't forget the header for loading packages:

\\usepackage{tikz}
\\usetikzlibrary{arrows,decorations.pathmorphing,backgrounds,positioning,fit,petri}


By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Dec 2017
Last update: Dec/19/2017
'''

import os
import numpy as np


class MahmoudTikzDirectedGraph:

    def __init__(self, inter_name, num_lanes, ppi):
        self.inter_name = inter_name
        self.num_lanes = num_lanes
        self.pli = ppi

    def set_phase_graph(self):
        self.filepath = os.path.join('log/' + self.inter_name, 'phases_graph.txt')
        if os.path.exists(self.filepath):
            self._f = open(self.filepath, 'w')
        else:
            self._f = open(self.filepath, 'x')

        self._make_header()

        s, e, gap = 0.7, 0.5, 2
        bl, tl, tr, br = [0, 0], [0, 6 * s + e], [5 * s + e, 5 * s + e], [5 * s + e, 0]

        x, y = bl
        x -= gap
        dis_lane, lane = 0, 0
        nt, ngap = 7, 5
        for i in range(nt):  # create left nodes
            if i < ngap:
                lane += 1
                self._f.write(
                    '\\node (l{:d}) at ({:2.2f},{:2.2f}) [circle,line width=0.8mm,draw,inner sep=0pt,minimum size=6mm] {{$l_{{{:d}}}$}};\n'.format(
                        lane, x, y, lane))
            else:
                dis_lane += 1
                self._f.write(
                    '\\node (d{:d}) at ({:2.2f},{:2.2f}) [rectangle,draw,inner sep=0pt,minimum size=6mm] {{$d_{{{:d}}}$}};\n'.format(
                        dis_lane, x, y, dis_lane))
            if i != ngap - 1:
                y += s
            else:
                y += s + e

        x, y = tl
        y += gap
        nt, ngap = 6, 4
        for i in range(nt):  # create top nodes
            if i < ngap:
                lane += 1
                self._f.write(
                    '\\node (l{:d}) at ({:2.2f},{:2.2f}) [circle,line width=0.8mm,draw,inner sep=0pt,minimum size=6mm] {{$l_{{{:d}}}$}};\n'.format(
                        lane, x, y, lane))
            else:
                dis_lane += 1
                self._f.write(
                    '\\node (d{:d}) at ({:2.2f},{:2.2f}) [rectangle,draw,inner sep=0pt,minimum size=6mm] {{$d_{{{:d}}}$}};\n'.format(
                        dis_lane, x, y, dis_lane))
            if i != ngap - 1:
                x += s
            else:
                x += s + e

        x, y = tr
        x += gap
        nt, ngap = 5, 3
        for i in range(nt):  # create right nodes
            if i < ngap:
                lane += 1
                self._f.write(
                    '\\node (l{:d}) at ({:2.2f},{:2.2f}) [circle,line width=0.8mm,draw,inner sep=0pt,minimum size=6mm] {{$l_{{{:d}}}$}};\n'.format(
                        lane, x, y, lane))
            else:
                dis_lane += 1
                self._f.write(
                    '\\node (d{:d}) at ({:2.2f},{:2.2f}) [rectangle,draw,inner sep=0pt,minimum size=6mm] {{$d_{{{:d}}}$}};\n'.format(
                        dis_lane, x, y, dis_lane))
            if i != ngap - 1:
                y -= s
            else:
                y -= s + e

        x, y = br
        y -= gap
        nt, ngap = 6, 4
        for i in range(nt):  # create bottom nodes
            if i < ngap:
                lane += 1
                self._f.write(
                    '\\node (l{:d}) at ({:2.2f},{:2.2f}) [circle,line width=0.8mm,draw,inner sep=0pt,minimum size=6mm] {{$l_{{{:d}}}$}};\n'.format(
                        lane, x, y, lane))
            else:
                dis_lane += 1
                self._f.write(
                    '\\node (d{:d}) at ({:2.2f},{:2.2f}) [rectangle,draw,inner sep=0pt,minimum size=6mm] {{$d_{{{:d}}}$}};\n'.format(
                        dis_lane, x, y, dis_lane))
            if i != ngap - 1:
                x -= s
            else:
                x -= s + e

        self.lane2lane = {1: [(8, '[out=0,in=90]')], 2: [(6, '[out=0,in=180]')], 3: [(5, '[out=0,in=180]')],
                          4: [(4, '[out=0,in=270]')], 5: [(3, '[out=0,in=270]')], 6: [(2, '[out=270,in=0]')],
                          7: [(8, '[out=270,in=90]')], 8: [(7, '[out=270,in=90]')], 9: [(5, '[out=270,in=180]')],
                          10: [(2, '[out=180,in=0]'), (4, '[out=180,in=270]')], 11: [(1, '[out=180,in=0]')],
                          12: [(7, '[out=180,in=90]')], 13: [(4, '[out=90,in=270]'), (6, '[out=90,in=180]')],
                          14: [(3, '[out=90,in=270]')], 15: [(2, '[out=90,in=0]')], 16: [(1, '[out=90,in=0]')]}

        for l, dis_lane in self.lane2lane.items():
            for d in dis_lane:
                self._f.write('\\draw[->,>=stealth,line width = 3pt] (l{:d}) {:s} to (d{:d});\n'.format(l, d[1], d[0]))

        self._closefile(self._f)

    def set_mcf_orig(self):
        self.filepath = os.path.join('log/' + self.inter_name, 'mcf_graph.txt')
        if os.path.exists(self.filepath):
            self._f = open(self.filepath, 'w')
        else:
            self._f = open(self.filepath, 'x')

        self._make_header()
        self._add_nodes()
        self._add_arcs()

        self._closefile(self._f)

    def _make_header(self):
        string = '\\begin{tikzpicture}\n'
        self._f.write(string)

    def _add_nodes(self):
        self._f.write('%NODES\n')

        H = 3  # height of layers
        W = (21 - 2 * 2.54) / self.num_lanes  # width of steps

        x_left = self.comp_x_left(self.num_lanes, W)
        x = x_left
        y = 4 * H
        for l in range(self.num_lanes):
            lane = l + 1
            self._f.write(
                '\\node (l{:d}) at ({:2.2f},{:2.2f}) [circle,draw,inner sep=0pt,minimum size=6mm] {{$l_{{{:d}}}$}};\n'.format(
                    lane, x, y, lane))
            self._f.write('\\node [black,above] at (l{:d}.north) {{{{\\tiny $(+d_{{{:d}}})$}}}};\n'.format(lane, lane))
            x += W

        W = (21 - 2 * 2.54) / len(self.pli)  # width of steps
        x_left = self.comp_x_left(len(self.pli), W)
        x = x_left
        y = 2 * H
        for ph in range(len(self.pli)):
            phase = ph + 1
            self._f.write(
                '\\node (p{:d}) at ({:2.2f},{:2.2f}) [circle,draw,inner sep=0pt,minimum size=6mm] {{$p_{{{:d}}}$}};\n'.format(
                    phase, x, y, phase))
            self._f.write(
                '\\node (pp{:d}) at ({:2.2f},{:2.2f}) [circle,draw,inner sep=0pt,minimum size=6mm] {{$p_{{{:d}}}^\prime$}};\n'.format(
                    phase, x, y - H, phase))
            x += W
        self._f.write(
            '\\node (sink) at ({:2.2f},{:2.2f}) [circle,draw,inner sep=1.5pt,minimum size=6mm] {{$sink$}};\n'.format(
                0, 0))
        self._f.write('\\node [black,below] at (sink.south) {{$(-\\sum_{{l=1}}^{{16}} d_{{l}})$}};\n')

    def comp_x_left(self, n, W):
        if n % 2 == 0:
            return -1 * W * n // 2
        else:
            return -1 * W * n // 2 + W / 2

    def _add_arcs(self):
        # \draw[->, line width = 1.8pt] FYI
        self._f.write('%ARCS\n')
        # add lane to phase arcs
        for p in range(len(self.pli)):
            for l in range(self.num_lanes):
                if self.pli[p, l]:
                    self._f.write('\\draw[->,> = latex,semithick] (l{:d}) to (p{:d});\n'.format(l + 1, p + 1))

        max_ph_size = max(np.sum(self.pli, 1))
        for p in range(len(self.pli)):
            self._f.write(
                '\\draw[->,> = latex,semithick,bend right=20] (p{:d}) to node [below,align=center,rotate=90,midway] {{{{\\tiny ${:d}/\\infty$}}}} (pp{:d});\n'.format(
                    p + 1, int(1 + max_ph_size - sum(self.pli[p]) + 1), p + 1))

            self._f.write(
                '\\draw[->,> = latex,semithick,bend left=20] (p{:d}) to node [above,align=center,rotate=90,midway] {{{{\\tiny $1/1$}}}} (pp{:d});\n'.format(
                    p + 1, p + 1))
            self._f.write('\\draw[->,> = latex,semithick] (pp{:d}) to [out=270,in={:2.2f}](sink);\n'.format(p + 1,
                                                                                                            180 - (
                                                                                                                    180 / (
                                                                                                                    len(
                                                                                                                        self.pli) - 1)) * p))

    def _closefile(self, file):
        file.write('\end{tikzpicture}')
        file.close()
