from bokeh.plotting import figure, ColumnDataSource
from bokeh.io import export_svgs, show
from bokeh.models import HoverTool


class MahmoudVisTrj:
    def __init__(self, lane):
        hover = HoverTool(tooltips=[
            ("index", "$index"),
            ("time", "$x{0.0 a} sec"),
            ("distance", "$y{0.0 a} m"),
        ])
        self.fig = figure(width=500, height=500, tools=[hover],
                          title="Mouse over the dots")
        self.fig.title.text = "Time-Space Diagram: Lane " + str(lane)
        self.fig.title.align = "center"
        self.fig.output_backend = "svg"

    def plotrj(self, t, d):
        source = ColumnDataSource(data=dict(
            x=t, y=d,
        ))
        self.fig.line('x', 'y', line_width=3, source=source)
        show(self.fig)

    def exprtrj(self):
        export_svgs(self.fig, filename="trj.svg")

# import matplotlib.pyplot as plt
# from numpy import sqrt as np
#
#
# class MahmoudVisual:
#     def __init__(self, nl):
#         '''
#
#         :param NL: number of lanes
#         '''
#
#         self.trjfig = plt.figure()
#
#         self.m, self.n = int(np.sqrt(nl)), int(np.sqrt(nl))
#         while self.m * self.n != nl:
#             self.m -= 1
#             self.n = nl // self.m
#
#     def plotrj(self, t, d, l):
#         plt.subplot(self.m, self.n, l)  # todo change the 3 to general index
#         plt.plot(t, d, 'ko-')
#         plt.title('A tale of 2 subplots')
#         plt.ylabel('Damped oscillation')
#
#     # def exprtrj(self, fig):
#     #     export_svgs(self.fig, filename="trj.svg")
#
#     def makeplt(self):
#         self.trjfig.show()
