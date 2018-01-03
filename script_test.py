from src.inpt.sim import MahmoudSimulator
from src.inter.inter import MahmoudIntersection
from src.trj.trjopt import MahmoudAVO
from src.inter.mcfopt import MahmoudSigNet
from src.inter.veh import MahmoudLanes, MahmoudVehicle
from src.vis.tikzpans import MahmoudTikZpanels, MahmoudTikzDirectedGraph

lanes = MahmoudLanes(1)

lanes.vehlist[0].add_last(MahmoudVehicle('234', 0, 0, 15, 150, 15))
fol_veh = lanes.vehlist[0].last()  # this would be lead for added vehicle
for k in range(3, 4):
    trjoptimizer = MahmoudAVO(None, fol_veh.element(), gs=19, gt=86400, fdeg=0, vmax=15, vcont=0)
    trjoptimizer.ctrl_get_one_comp_trj()


#######################################################################################
