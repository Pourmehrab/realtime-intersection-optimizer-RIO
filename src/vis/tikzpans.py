'''
Goal: Make phases panels for publication

Don't forget the header for loading packages:

\\usepackage{tikz}
\\usetikzlibrary{arrows,decorations.pathmorphing,backgrounds,positioning,fit,petri}


By:     Mahmoud Pourmehrab
E-mail: mpourmehrab@ufl.edu
Date:        Dec 2017
Last update: Dec/19/2017
'''

import os

from src.vis.tikz import TikzDirectedGraph


class TikZpanels(TikzDirectedGraph):

    def __init__(self, inter_name, num_lanes, ppi):

        super().__init__(inter_name, num_lanes, ppi)

        self.filepath = os.path.join('log/' + self.inter_name, 'inter_panel_graph.txt')
        if os.path.exists(self.filepath):
            self._f = open(self.filepath, 'w')
        else:
            self._f = open(self.filepath, 'x')

        self._make_header()

        s, e, gap = 0.7 / 2, 0.5 / 2, 2 / 2
        bl, tl, tr, br = [0, 0], [0, 6 * s + e], [5 * s + e, 5 * s + e], [5 * s + e, 0]
        blp, tlp, trp, brp = [0, 0], [0, 6 * s + e], [5 * s + e, 5 * s + e], [5 * s + e, 0]
        dis_lane, lane = 0, 0
        self.lane2lane = {1: [[8, '[out=0,in=90]']], 2: [[6, '[out=0,in=180]']], 3: [[5, '[out=0,in=180]']],
                          4: [[4, '[out=0,in=270]']], 5: [[3, '[out=0,in=270]']], 6: [[2, '[out=270,in=0]']],
                          7: [[8, '[out=270,in=90]']], 8: [[7, '[out=270,in=90]']], 9: [[5, '[out=270,in=180]']],
                          10: [[2, '[out=180,in=0]'], [4, '[out=180,in=270]']], 11: [[1, '[out=180,in=0]']],
                          12: [[7, '[out=180,in=90]']], 13: [[4, '[out=90,in=270]'], [6, '[out=90,in=180]']],
                          14: [[3, '[out=90,in=270]']], 15: [[2, '[out=90,in=0]']], 16: [[1, '[out=90,in=0]']]}

        for ph in range(len(ppi)):
            W, H = 4, 4  # distance between panels
            j = 1 * (ph % 4)
            i = -1 * (ph // 4)
            self._f.write('%panel: ({:d},{:d})\n'.format(i, j))
            blp[0] = bl[0] + j * W
            tlp[0] = tl[0] + j * W
            trp[0] = tr[0] + j * W
            brp[0] = br[0] + j * W
            blp[1] = bl[1] + i * H
            tlp[1] = tl[1] + i * H
            trp[1] = tr[1] + i * H
            brp[1] = br[1] + i * H

            dis_lane, lane = self._make_panel(s, e, gap, blp, tlp, trp, brp, dis_lane, lane, ph)

        self._closefile(self._f)

    def _make_panel(self, s, e, gap, bl, tl, tr, br, dis_lane, lane, ph):
        x, y = bl
        x -= gap
        nt, ngap = 7, 5
        base_dis_lane, base_lane = dis_lane, lane
        for i in range(nt):  # create left nodes
            if i < ngap:
                lane += 1
                self._f.write(
                    '\\node (l{:d}) at ({:2.2f},{:2.2f}) [circle,inner sep=0pt,minimum size=6mm] {{}};\n'.format(
                        lane, x, y, lane))
            else:
                dis_lane += 1
                self._f.write(
                    '\\node (d{:d}) at ({:2.2f},{:2.2f}) [rectangle,inner sep=0pt,minimum size=6mm] {{}};\n'.format(
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
                    '\\node (l{:d}) at ({:2.2f},{:2.2f}) [circle,inner sep=0pt,minimum size=6mm] {{}};\n'.format(
                        lane, x, y, lane))
            else:
                dis_lane += 1
                self._f.write(
                    '\\node (d{:d}) at ({:2.2f},{:2.2f}) [rectangle,inner sep=0pt,minimum size=6mm] {{}};\n'.format(
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
                    '\\node (l{:d}) at ({:2.2f},{:2.2f}) [circle,inner sep=0pt,minimum size=6mm] {{}};\n'.format(
                        lane, x, y, lane))
            else:
                dis_lane += 1
                self._f.write(
                    '\\node (d{:d}) at ({:2.2f},{:2.2f}) [rectangle,inner sep=0pt,minimum size=6mm] {{}};\n'.format(
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
                    '\\node (l{:d}) at ({:2.2f},{:2.2f}) [circle,inner sep=0pt,minimum size=6mm] {{}};\n'.format(
                        lane, x, y, lane))
            else:
                dis_lane += 1
                self._f.write(
                    '\\node (d{:d}) at ({:2.2f},{:2.2f}) [rectangle,inner sep=0pt,minimum size=6mm] {{}};\n'.format(
                        dis_lane, x, y, dis_lane))
            if i != ngap - 1:
                x -= s
            else:
                x -= s + e

        for l, dis in self.lane2lane.items():
            if self.ppi[ph, l - 1]:
                for d in dis:
                    self._f.write(
                        '\\draw[->,>=stealth,line width = 3pt] (l{:d}) {:s} to (d{:d});\n'.format(base_lane + l, d[1],
                                                                                                  base_dis_lane + d[0]))
            else:
                for d in dis:
                    self._f.write(
                        '\\draw[->,>=stealth,line width = 1pt,gray] (l{:d}) {:s} to (d{:d});\n'.format(base_lane + l,
                                                                                                       d[1],
                                                                                                       base_dis_lane +
                                                                                                       d[0]))

        return dis_lane + 1, lane + 1
