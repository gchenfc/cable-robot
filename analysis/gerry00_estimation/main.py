import numpy as np
import calibrate_orig
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

def rot_xyz(rots):
    rots = Rotation.from_matrix(rots @ np.linalg.inv(rots[0])).as_rotvec()
    rots[rots < -3] += np.pi
    rots[rots > 3] -= np.pi
    return rots


def load_all(fname, max_t=None):
    # cdpr log
    t, est, des, lens, vels = calibrate_orig.load_log(fname + '.txt')
    starti = calibrate_orig.start_index(t, est)
    t, est, des, lens, vels = t[starti:], est[starti:], des[starti:], lens[starti:], vels[starti:]
    t -= t[0]
    if max_t is not None:
        endi = np.argwhere(t > max_t)[0][0]
        t, est, des, lens, vels = t[:endi], est[:endi], des[:endi], lens[:endi], vels[:endi]

    # mocap
    t_act, act, act_rot, l_gt = calibrate_orig.load_mocap(fname + '.csv')
    # t_act, act, act_rot, l_gt = calibrate_orig.load_mocap('rot_x-y-z_(y=gravity).csv')

    starti = calibrate_orig.start_index(t_act, act)-10
    t_act, act, act_rot, l_gt = t_act[starti:], act[starti:], act_rot[starti:], l_gt[starti:]
    t_act -= t_act[0]
    if max_t is not None:
        endi = np.argwhere(t_act > max_t)[0][0]
        t_act, act, act_rot, l_gt = t_act[:endi], act[:endi], act_rot[:endi], l_gt[:endi]

    # re-interpolate mocap timestamps
    t_off = calibrate_orig.align_time(t, est, t_act, act)
    print('offset time:', t_off)
    interp = lambda act: np.vstack(
        tuple(np.interp(t + t_off, t_act, act_dim) for act_dim in act.T)).T
    act_interp = interp(act)
    act_rot_interp = Rotation.from_rotvec(interp(rot_xyz(act_rot))).as_matrix()
    act_lens = interp(l_gt)

    # return
    print(act_interp.shape, act_rot_interp.shape, est.shape, des.shape, lens.shape, vels.shape)
    return t, act_interp, est, des, act_lens, lens, vels

def main(fname='13_concentric_diamonds_1mps_cal'):
    t, x_gt, x_est, x_des, l_gt, l_meas, ldot_meas = load_all(fname, max_t=24.5)
    l_err = l_meas - l_gt

    fig1 = plt.figure()
    plt.plot(x_gt[:, 0], x_gt[:, 1])
    plt.plot(x_est[:, 0], x_est[:, 1])
    plt.legend(('Ground Truth', 'Un-calibrated Estimate'))
    plt.axis('equal')
    plt.xlim([0, 2.9])
    plt.ylim([0, 2.3])
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.grid(True)
    plt.title('Trajectory')

    fig = plt.figure(figsize=(15, 11))
    axes = fig.subplots(4, 3, sharex='col', sharey='col')
    for ax in axes.flatten():
        ax.grid(True)
        ax.set_axisbelow(True)
    for i in range(4):
        axes[i, 0].plot(t, l_gt[:, i])
        axes[i, 0].plot(t, l_meas[:, i], ':')
        axes[i, 1].scatter(l_err[:, i], l_gt[:, i], s=1)
        axes[i, 2].hist(l_err[:, i], 30)
    axes[0, 0].get_shared_y_axes().join(axes[0, 0], axes[0, 1])
    # fig.text(0.5, 0.04, 'common X', ha='center')
    fig.text(0.04, 0.5, 'Cable Length (m)', va='center', rotation='vertical', fontsize=30)
    fig.text(0.90, 0.5, 'Histogram Occurences', va='center', rotation=-90, fontsize=20)
    axes[-1, 0].set_xlabel('t (s)', fontsize=20)
    axes[-1, 1].set_xlabel('Cable Length \nMeasurement Error (m)', fontsize=20)
    axes[-1, 2].set_xlabel('Cable Length \nMeasurement Error (m)', fontsize=20)
    for i in range(4):
        axes[i, 0].set_ylabel('Cable {}'.format(i), fontsize=15)
    plt.suptitle('Cable Length Measurement Statistics', fontsize=40)
    
    fig1.savefig('{}_trajectory.png'.format(fname))
    fig.savefig('{}_measurement_stats.png'.format(fname))
    plt.show()

if __name__ == '__main__':
    main()
