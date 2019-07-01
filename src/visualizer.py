import matplotlib.pylab as pylab

params = {
    # 'legend.fontsize': 'xx-small',
    # 'figure.figsize': (15, 5),
    'axes.labelsize': 4,
    'axes.titlesize': 5,
    'xtick.labelsize': 4,
    'ytick.labelsize': 4}
pylab.rcParams.update(params)
import matplotlib.pyplot as plt
import numpy as np
import os

def find_pannel_matplot(lane, _num_cols):
    return lane // _num_cols, lane % _num_cols


def plot_SPaT_and_trajs(lanes, signal, intersection, absolute_time, show=False, save_dir=''):
    num_lanes = len(lanes.vehlist)
    min_dist_to_stop_bar = intersection._inter_config_params.get('min_dist_to_stop_bar')

    root = np.sqrt(num_lanes)
    _num_rows = int(np.floor(root))
    if root - _num_rows > 0.001:
        _num_rows = int(round(root, 0))
        _num_cols = num_lanes // _num_rows
        if _num_rows * _num_cols < num_lanes:
            _num_cols += 1
    else:
        _num_cols = _num_rows
    
    _fig_matplotlib, _ax_matplotlib = plt.subplots(nrows=_num_rows, ncols=_num_cols, sharex=True, sharey=True)
    
    num_phases_in_SPaT = len(signal.SPaT_sequence)

    for lane, ax in enumerate(_fig_matplotlib.axes):
        if lane == num_lanes:
            break
        ax.set_title('Lane ' + str(lane + 1))
        i, j = find_pannel_matplot(lane, _num_cols)
        if i == _num_rows - 1:
            ax.set_xlabel('Time (second)')
        if j == 0:
            ax.set_ylabel('Distance to Stop Bar (m)')
        ax.set_facecolor('#d4d4d4' if lane % 2 == 1 else 'w')

        for p in range(num_phases_in_SPaT):
            phase = signal.SPaT_sequence[p]
            color = 'g' if lane in signal._phase_lane_incidence[phase] else 'r'
            _ax_matplotlib[i, j].plot([signal.SPaT_start[p], signal.SPaT_end[p]], [0] * 2, color=color, linewidth=2)
            _ax_matplotlib[i, j].plot([signal.SPaT_start[p], signal.SPaT_end[p]], [min_dist_to_stop_bar] * 2, color='k',
                                      linewidth=1, linestyle=':')
        for veh in lanes.vehlist[lane]:
            color = 'b' if veh.veh_type == 1 else 'k'
            s, e = veh.first_trj_point_indx, veh.last_trj_point_indx + 1
            _ax_matplotlib[i, j].plot(veh.trajectory[0, s:e], veh.trajectory[1, s:e], color=color, linewidth=1)
            arr_time, arr_dist, dep_time, dep_dist = veh.trajectory[0, s], veh.trajectory[1, s], veh.trajectory[
                0, e - 1], veh.trajectory[1, e - 1]
            ax.text(arr_time, arr_dist, '{:2.1f}m{:2.1f}s'.format(arr_dist, dep_time)
                    , horizontalalignment='center', fontsize=8)  # distance/departure time
            # traj = get_veh_traj(lanes.vehlist, lane, 0)

        ax.tick_params(axis='both', which='major', labelsize=6)

    plt.suptitle('time: {:2.1f} sec'.format(absolute_time))
    plt.xticks([t for t in signal.SPaT_start] + [signal.SPaT_end[-1]], rotation=270)
    if save_dir != '':
        plt.savefig(os.path.join(save_dir, '{}.png'.format(int(1000 * absolute_time))))
    if show:
        plt.show()
    plt.clf() # clear

def get_veh_traj(vehlist, lane, veh_indx):
    veh = vehlist[lane][veh_indx]
    s, e = veh.first_trj_point_indx, veh.last_trj_point_indx + 1
    return veh.trajectory[:, s:e]


class VisualizeSpaceTime:
    def __init__(self, num_lanes):
        '''

        :param num_lanes: should be more than 2
        :param det_range: in meters
        '''

        # Initialize MatPlotLip
        root = np.sqrt(num_lanes)
        self._num_rows = int(root)
        if root - self._num_rows > 0.001:
            self._num_rows = int(round(root, 0))
            self._num_cols = num_lanes // self._num_rows
            if self._num_rows * self._num_cols < num_lanes:
                self._num_cols += 1
        else:
            self._num_cols = self._num_rows

        self._fig_matplotlib, self._ax_matplotlib = plt.subplots(
            nrows=self._num_rows, ncols=self._num_cols,
            sharex=True, sharey=True)

        for lane, ax in enumerate(self._fig_matplotlib.axes):
            ax.set_title('Trajectories in Lane ' + str(lane))
            i, j = self.find_pannel_matplot(lane)
            if i == self._num_rows - 1:
                ax.set_xlabel('Time (second)')

            if j == 0:
                ax.set_ylabel('Distance to Stop Bar (m)')

            ax.set_facecolor('#d4d4d4' if lane % 2 == 1 else 'w')

        plt.figure(dpi=1500)

    def find_pannel_matplot(self, lane):
        return lane // self._num_cols, lane % self._num_cols

    def add_multi_trj_matplotlib(self, veh, lane):
        t, d, s = veh.trajectory[:, veh.first_trj_point_indx: veh.last_trj_point_indx]

        # lane is zero based
        i, j = self.find_pannel_matplot(lane)
        color = 'green' if veh.veh_type == 1 else 'red'  # todo color gradient line on speed
        # linestyle = 'solid ' if vehicle_type == 1 else 'dashed' # then add , linestyle=linestyle to below
        line = self._ax_matplotlib[i, j].plot(t, d, color=color, linewidth=1)

        # line.pop(0).remove()

    def export_matplot(self, sc, det_range, sim_start_time, sim_end_time, format='jpg'):  # 'pdf'
        xmin, xmax, ymin, ymax = sim_start_time, sim_end_time, det_range, -50  # todo fix ymin
        for lane, ax in enumerate(self._fig_matplotlib.axes):
            ax.axis([xmin, xmax, ymin, ymax])

        plt.draw()
        plt.show()
        self._fig_matplotlib.savefig('matplot_trjs_sc_' + str(sc) + '.' + format, format=format)

if __name__ == '__main__':
    from moviepy.editor import ImageSequenceClip
    import glob
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--img-dir', default='', type=str)
    parser.add_argument('--format', default='*.png', type=str)
    parser.add_argument('--name', default='out.gif', type=str)

    args = parser.parse_args()

    files = glob.glob(os.path.join(args.img_dir, args.format))
    
    def get_name(x):
        bn = os.path.basename(x)
        return int(bn.split('.')[0])
    files = sorted(files, key = lambda x: get_name(x))

    out_name = os.path.join(args.img_dir, args.name)
    clip = ImageSequenceClip(files, fps=1)
    clip.write_gif("{}".format(out_name), fps=1)
