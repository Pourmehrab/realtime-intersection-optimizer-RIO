from src.inpt.sim import Simulator
from src.inter.inter import Intersection
from src.trj.trjopt import Connected
from src.inter.mcfopt import SigMinCostNet
from src.inter.veh import Lanes, Vehicle
from src.vis.tikzpans import TikZpanels, TikzDirectedGraph

lanes = Lanes(1)

lanes.vehlist[0].add_last(Vehicle('234', 0, 0, 15, 150, 15))
fol_veh = lanes.vehlist[0].last()  # this would be lead for added vehicle
for k in range(3, 4):
    trjoptimizer = Connected(None, fol_veh.element(), gs=19, gt=86400, fdeg=0, vmax=15, vcont=0)
    trjoptimizer.ctrl_get_one_comp_trj()


#######################################################################################
