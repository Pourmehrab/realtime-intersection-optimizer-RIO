####################################
# File name: trj_planner.py        #
# Author: Mahmoud Pourmehrab       #
# Email: mpourmehrab@ufl.edu       #
# Last Modified: Feb/26/2018       #
####################################


from src.trj.trj import Trajectory
from src.trj.trjopt import Connected
from src.trj.trjest import Conventional


def trj_planner(signal, lanes, num_lanes, max_speed):
    for lane in range(num_lanes):

        if bool(lanes.vehlist[lane]):  # lane has some vehicles

            lead_veh_indx = 0
            phase_index = signal.get_next_phase_indx(0, lane)  # gives the first phase INDEX that serves this vehicle

            if phase_index < 0:
                raise Exception('SPaT decision failed to even serve first vehicle in lane {:d}'.format(lane))

            lead_veh = lanes.vehlist[lane][lead_veh_indx]

            dep_time = signal.SPaT_start[phase_index] + Trajectory.LAG
            dep_time = max(dep_time, lead_veh.earlst)

            if lead_veh.veh_type == 1:
                trj_planner = Connected(None, lead_veh, gs=dep_time, vmax=max_speed)
                # trj_planner.insight()  # optional: if want to print some overview of follower vehicle
                trj_planner.solve(0)  # pass 0 for lead vehicle
            else:
                trj_planner = Conventional(None, lead_veh, gs=dep_time, vmax=max_speed)
                # trj_planner.insight()  # optional: if want to print some overview of follower vehicle
                trj_planner.solve(0)  # pass 0 for lead vehicle

            follower_veh_indx = 1
            while follower_veh_indx < len(lanes.vehlist[lane]):
                follower_veh = lanes.vehlist[lane][follower_veh_indx]

                dep_time = max(dep_time + Trajectory.SAT, follower_veh.earlst)

                if dep_time > signal.SPaT_end[phase_index]:
                    phase_index = signal.get_next_phase_indx(phase_index, lane)
                    if phase_index < 0:
                        raise Exception('SPaT decision failed to serve a follower vehicle in lane {:d}'.format(lane))
                    dep_time = signal.SPaT_start[phase_index] + Trajectory.LAG

                if lead_veh.veh_type == 1:
                    trj_planner = Connected(lead_veh, follower_veh, gs=dep_time, vmax=max_speed)
                    # trj_planner.insight()  # optional: if want to print some overview of follower vehicle
                    trj_planner.solve(1)  # pass 0 for lead vehicle
                else:
                    trj_planner = Conventional(lead_veh, follower_veh, gs=dep_time, vmax=max_speed)
                    # trj_planner.insight()  # optional: if want to print some overview of follower vehicle
                    trj_planner.solve(1)  # pass 0 for lead vehicle

                lead_veh = lanes.vehlist[lane][follower_veh_indx]
                follower_veh_indx += 1

