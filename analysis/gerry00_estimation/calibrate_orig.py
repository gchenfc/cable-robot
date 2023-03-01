import numpy as np
import matplotlib.pyplot as plt
import csv
from scipy.spatial.transform import Rotation
import scipy.optimize
from scipy.signal import savgol_filter
import scipy.io


def load_log(fname):
    data = np.loadtxt(fname,
                      comments="S",
                      usecols=(1, 2, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15, 17, 18, 19, 20, 22, 23, 24,
                               25))

    t = np.arange(0, data.shape[0] / 100, 0.01)
    invalid = data[:, 2] == 0
    # data = data[data[:, 2] != 0, :]
    # t = t[data[:, 2] != 0]
    # t -= t[0]

    est = data[:, :2]
    des = data[:, 2:4]
    errors = data[:, 4::4]
    states = data[:, 5::4]
    lens = -data[:, 6::4]
    vels = -data[:, 7::4]

    des[invalid, :] = np.nan

    return t, est, des, lens, vels


def transform_from_4_positions(bl, br, tl, tr):
    wPl = bl
    x_axis = br - bl
    y_axis = tl - bl
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)
    wRl = np.vstack((x_axis, y_axis, z_axis)).T
    wTl = np.zeros((4, 4))
    wTl[:3, :3] = wRl
    wTl[:3, 3] = wPl
    wTl[3, 3] = 1

    # unit test
    to_world = lambda pos: (wTl @ np.array([*pos, 1]).T)[:3]
    assert np.allclose(to_world(np.array([0, 0, 0])), bl, 1e-5), "origin of transform not correct"
    assert np.allclose(to_world(np.array([1, 0, 0]) * np.linalg.norm(br - bl)), br,
                       1e-5), "x-axis of transform not correct"
    assert np.allclose(to_world(np.array([0, 1, 0]) * np.linalg.norm(tl - bl)), tl,
                       1e-5), "y-axis of transform not correct"

    return wTl


def load_mocap(fname, ee_name='Rigid Body 2', frame_name='Rigid Body 1'):
    with open(fname, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # format stuff
        next(reader)  # blank
        types = next(reader)  # Rigid Body, Rigid Body Marker, Marker
        names = next(reader)  # Rigid Body 1_2
        hashes = next(reader)
        descs = next(reader)  # Position or Rotation
        dims = next(reader)  # X, Y, Z, W

    data = np.genfromtxt(fname, delimiter=',', skip_header=7)
    framei = data[:, 0]
    time = data[:, 1]

    def col(name, desc, dim):
        cols = [
            i for i in range(len(names)) if names[i] == name and descs[i] == desc and dims[i] == dim
        ]
        if len(cols) == 1:
            return cols[0]
        raise Exception("Problem: ", cols)

    def rigid_body_pose(name):
        xyz_cols = [col(name, 'Position', dim) for dim in 'XYZ']
        quat_cols = [col(name, 'Rotation', dim) for dim in 'XYZW'] # scalar-last format XYZW
        pos = data[:, xyz_cols]
        rot = Rotation.from_quat(data[:, quat_cols]).as_matrix()
        return pos, rot

    def marker_pos(name):
        xyz_cols = [col(name, 'Position', dim) for dim in 'XYZ']
        return data[:, xyz_cols]

    def append_1(pos):
        return np.vstack((pos.T, np.ones((1, pos.shape[0]))))
    def posrot_to_T(pos, rot):
        return np.concatenate(
            (
                np.concatenate((rot, np.zeros((rot.shape[0], 1, 3))), axis=1),  #
                append_1(pos).T.reshape(-1, 4, 1)),
            axis=2)

    # End effector
    ee_pos, ee_rot = rigid_body_pose(ee_name)
    wPe = append_1(ee_pos)
    wRe = ee_rot
    eRw = wRe.swapaxes(1, 2)
    wTe = posrot_to_T(ee_pos, wRe)
    eTw = posrot_to_T(-np.einsum('tij,tj->ti', eRw, ee_pos), eRw)
    # End effector marker locations
    ePm = np.zeros((4, 3))
    for markeri in range(4):
        xyz_cols = [col('{}_{}'.format(ee_name, markeri+1), 'Position', dim) for dim in 'XYZ']
        markerpos = append_1(data[:, xyz_cols])
        relpos = np.einsum('ijk,ki->ij', eTw, markerpos)
        bad = np.argwhere(np.linalg.norm(relpos - relpos[0], axis=1) > 1e-3)
        if len(bad) > 10:
            raise Exception('too many bad EE readings')
        relpos[bad, :] = np.nan
        ePm[markeri, :] = np.nanmean(relpos[:, :3], axis=0)
    print('Marker positions in the end effector frame:\n', ePm)
    # End effector mounting locations
    REAL_DIMS = np.array([131.5, 120]) * 1e-2
    ePm = ePm[[3, 1, 2, 0], :] # TODO: don't hardcode this
    box_dims = np.sqrt(np.square(ePm.reshape((-1, 3, 1)) - ePm.T.reshape((1, 3, -1))).sum(axis=1))
    assert np.allclose(0.120, box_dims[0, 1], atol=3e-3) and \
        np.allclose(0.120, box_dims[2, 3], atol=3e-3), "EE markers don't match expected"
    assert np.allclose(0.1315, box_dims[0, 3], atol=3e-3) and \
        np.allclose(0.1315, box_dims[1, 2], atol=3e-3), "EE markers don't match expected"
    bot, top = [0, 3], [1, 2]
    ext = 26e-3
    tmp = np.mean(ePm[bot, :], axis=0), np.mean(ePm[top, :], axis=0)
    ePm[bot, :] = (ePm[bot, :] - tmp[0]) * (box_dims[0, 3] + ext * 2) / box_dims[0, 3] + tmp[0]
    ePm[top, :] = (ePm[top, :] - tmp[1]) * (box_dims[1, 2] + ext * 2) / box_dims[1, 2] + tmp[1]

    # Frame
    marker_names = list(reversed(sorted(list(set([name for name in names if 'Marker_' in name])))))
    print(marker_names)
    marker_positions = [marker_pos(name) for name in marker_names]
    mean_positions = [np.nanmean(pos, axis=0) for pos in marker_positions]
    corner_info = np.array(mean_positions < np.nanmean(ee_pos, axis=0))
    bl = np.argwhere(corner_info[:, 2] & corner_info[:, 1])[0][0]
    br = np.argwhere(np.logical_not(corner_info[:, 2]) & corner_info[:, 1])[0][0]
    tl = np.argwhere(corner_info[:, 2] & np.logical_not(corner_info[:, 1]))[0][0]
    tr = np.argwhere(np.logical_not(corner_info[:, 2]) & np.logical_not(corner_info[:, 1]))[0][0]
    bl, br, tl, tr = mean_positions[bl], mean_positions[br], mean_positions[tl], mean_positions[tr]
    print('corner locs:', br - bl, tl - bl)
    wTf = transform_from_4_positions(bl, br, tl, br)
    fTw = np.linalg.inv(wTf)
    frame = fTw @ np.array([np.array([*corner, 1]) for corner in [br, tr, tl, bl]]).T
    frame = frame.T[:, :3]
    print('frame: \n', frame)

    #
    fPe = fTw @ wPe
    fPe[2, :] -= fPe[2, 0]

    # clean up
    print('wTf:\n', wTf)
    print('fTw:\n', fTw)
    fRe = fTw[:3, :3] @ wRe
    ee_rot = fRe @ np.linalg.inv(fRe[0]) @ np.eye(3)
    ee_rot_eul = Rotation.from_matrix(ee_rot).as_rotvec()
    ee_rot_eul -= ee_rot_eul[0]
    ee_rot_eul[ee_rot_eul > 3] -= np.pi
    ee_rot_eul[ee_rot_eul < -3] += np.pi
    smoothed = savgol_filter(ee_rot_eul, 301, 1, axis=0)
    bad = np.any(np.abs(ee_rot_eul - smoothed) > 0.1, axis=1) # mocap dropout
    print("removed: {:d} / {:d}".format(np.count_nonzero(bad), ee_rot_eul.shape[0]))
    ee_rot_eul[bad, :] = np.nan
    ee_rot_eul -= np.nanmean(ee_rot_eul, axis=0)
    fRe[bad, :, :] = np.nan

    # plt.plot(time, ee_rot_eul)
    # plt.plot(time, smoothed)
    # plt.legend(('x', 'y', 'z'))
    # plt.show()

    fPm = np.einsum('ij,tjk->tik', fTw, wTe @ append_1(ePm))
    lens = np.sqrt(np.sum(np.square(fPm[:, :3, :] - frame.T), axis=1))

    return time, fPe[:3, :].T, fRe, lens


def start_index(t, x):
    # xdot = np.diff(x, axis=0) / np.diff(t, axis=0).reshape(-1, 1)
    xdot = x - x[0]
    return np.argwhere(np.abs(xdot[:, 0]) > 0.02)[0][0]


# useful alignment functions
def align(gt, x1, *args, style='default'):
    basis_functions = lambda x, y: np.hstack(
        (np.ones(x.shape), x, y, x * x, x * y, y * y, x * x * x, x * x * y, x * y * y, y * y * y))
    transforms_inits = {
        'none': (
            lambda x, t: x,  #
            np.array([0, 0])),
        'default': (
            lambda x, t: x * t[0:2] + t[2:4],  #
            np.array([0.95, 0.9, 0.2, 0.15])),
        'nonlinear': (
            lambda x, t: basis_functions(x[:, 0:1], x[:, 1:2]) @ t.reshape((-1, 2)),  #
            np.array([[0.2, 0.15], [0.95, 0], [0, 0.9], *np.zeros((7, 2))]))
    }
    transform, init = transforms_inits[style]
    err = lambda t: np.linalg.norm(gt - transform(x1, t), 'fro')
    t_opt = scipy.optimize.minimize(err, init).x
    return t_opt, transform(x1, t_opt), *[transform(arg, t_opt) for arg in args]

def fk(lens, params):
    w, h = params[:2]
    multipliers = params[2:6]
    offsets = params[6:]
    lens = lens * multipliers + offsets
    # law of cosines
    a = lens[:, 1]
    b = w
    c = lens[:, 2]
    cosalpha = (np.square(b) + np.square(c) - np.square(a)) / (2 * b * c)
    cosalpha[cosalpha > 1] = 1
    x = c * cosalpha
    y = h - c * np.sqrt(1 - np.square(cosalpha))
    return np.vstack((x, y)).T

def fk2(lens, params):
    tl = params[0:2]
    tr = params[2:4]
    w = np.linalg.norm(tl - tr)
    multipliers = params[4:8]
    offsets = params[8:12]
    lens = lens * multipliers + offsets
    # law of cosines
    a = lens[:, 1]
    b = w
    c = lens[:, 2]
    cosalpha = (np.square(b) + np.square(c) - np.square(a)) / (2 * b * c)
    cosalpha[cosalpha > 1] = 1
    x = c * cosalpha
    y = c * np.sqrt(1 - np.square(cosalpha))
    # rotate & flip
    cos_th = (tr[0] - tl[0]) / w
    sin_th = (tr[1] - tl[1]) / w
    x_new = tl[0] + x * cos_th + y * sin_th
    y_new = tl[1] + x * sin_th - y * cos_th
    return np.vstack((x_new, y_new)).T

def fk3(lens, params):
    tl = params[0:2]
    tr = params[2:4]
    w = np.linalg.norm(tl - tr)
    multipliers_b = params[4:8]
    offsets = params[8:12]
    multipliers_m = params[12:16]
    lens = lens * (multipliers_m * lens + multipliers_b) + offsets
    # law of cosines
    a = lens[:, 1]
    b = w
    c = lens[:, 2]
    cosalpha = (np.square(b) + np.square(c) - np.square(a)) / (2 * b * c)
    cosalpha[np.abs(cosalpha) > 1] = 1
    x = c * cosalpha
    y = c * np.sqrt(1 - np.square(cosalpha))
    # rotate & flip
    cos_th = (tr[0] - tl[0]) / w
    sin_th = (tr[1] - tl[1]) / w
    x_new = tl[0] + x * cos_th + y * sin_th
    y_new = tl[1] + x * sin_th - y * cos_th
    return np.vstack((x_new, y_new)).T

def align2(gt, lens, style='calibrate1'):
    # params = [width, height, 4*length multpiliers, 4*length offsets]
    def ik(gt, params):
        w, h = params[:2]
        multipliers = params[2:6]
        offsets = params[6:]
        corners = np.array([[w, 0], [w, h], [0, h], [0, 0]]).T.reshape(1,2,4)
        return (np.linalg.norm(gt - corners, axis=1) - offsets) / multipliers
    fk_inits = {
        'calibrate1': (
            fk,  #
            np.array([2.9, 2.32, 1, 1, 1, 1, 0, 0, 0, 0])),
        'calibrate2': (
            fk2,  #
            np.array([0, 2.32, 2.9, 2.32, 1, 1, 1, 1, 0, 0, 0, 0])),
        'calibrate3': (
            fk3,  #
            np.array([0, 2.32, 2.86, 2.28, 1, 0.93, 1.02, 1, 0, 0.15, 0.15, 0, 0, 0.001, -0.02, 0]))
    }
    def another():
        err = lambda params: np.linalg.norm(ik(gt.reshape(-1, 2, 1), params) - lens, 'fro')
        params_opt = scipy.optimize.minimize(err, fk_inits['calibrate1'][1]).x
        print("ANOTHER")
        print(params_opt)
    fk_, params_guess = fk_inits[style]
    err = lambda params: np.linalg.norm(gt - fk_(lens, params), 'fro')
    bnds = [(mi, ma) for mi, ma in zip(params_guess-0.2, params_guess+0.2)]
    # params_opt = scipy.optimize.minimize(err, params_guess, bounds=bnds).x
    params_opt = scipy.optimize.minimize(err, params_guess).x
    print("optimization error: ", err(params_opt))
    # another()
    return params_opt, fk_(lens, params_opt)

def align_time(t, est, t_act, act):
    interp = lambda toff: np.vstack(tuple(np.interp(t + toff, t_act, act_dim) for act_dim in act[:, :2].T)).T
    err = lambda toff: np.linalg.norm(est - interp(toff), 'fro')
    params_opt = scipy.optimize.minimize(err, 0).x
    return params_opt

def main(name='ATL_filled_10mps2_1e4'):
    max_t = 67.7

    # cdpr log
    t, est, des, lens, vels = load_log(name + '.txt')
    starti = start_index(t, est)
    t, est, des, lens, vels = t[starti:], est[starti:], des[starti:], lens[starti:], vels[starti:]
    t -= t[0]
    # endi = np.argwhere(t > max_t)[0][0]
    # t, est, des, lens, vels = t[:endi], est[:endi], des[:endi], lens[:endi], vels[:endi]

    # mocap
    t_act, act, act_rot, frame = load_mocap(name + '.csv')
    starti = start_index(t_act, act)-10
    t_act, act, act_rot = t_act[starti:], act[starti:], act_rot[starti:]
    t_act -= t_act[0]
    # endi = np.argwhere(t_act > max_t)[0][0]
    # t_act, act, act_rot = t_act[:endi], act[:endi], act_rot[:endi]

    # re-interpolate mocap timestamps
    t_off = align_time(t, est, t_act, act)
    print('offset time:', t_off)
    act_interp = np.vstack(np.interp(t + t_off, t_act, act_dim) for act_dim in act[:, :2].T).T
    act_rot_interp = np.vstack(np.interp(t + t_off, t_act, act_dim) for act_dim in act_rot.T).T

    # alignment transform
    align_style = 'none'
    if 'calibrate' not in align_style:
        t_opt, est, des = align(act_interp, est, des, style=align_style)
        est_cal = est
        np.set_printoptions(suppress=True)
        print(t_opt.reshape((-1, 2)).round(4))
    else:
        t_opt, est_cal = align2(act_interp.copy(), lens, style=align_style)
        print(t_opt)
        print("static constexpr float kLenCorrectionParams[4][3] = {")
        for to in t_opt.reshape(-1, 4).T:
            print("    {{{:.14f}, {:.14f}, {:.14f}}},".format(to[3], to[1], to[2]))
        print("};")
        print("static constexpr float kMountPoints[4][2] = {")
        print("    {{kWidth, 0}}, {{{:.14f}, {:.14f}}}, {{{:.14f}, {:.14f}}}, {{0, 0}}}};".format(
            *t_opt[2:4], *t_opt[0:2]))
        print("Unit test:")
        ls = np.array([
            [0, 1., 2., 0.],  #
            [83., 2.5, 1.2, -123],
            [1., 0.5, 2.6, 3],
            [2., 3., 3., 4]
        ])
        Ts = fk3(ls, t_opt)
        print(-ls)
        print(Ts)

    # Error calculation
    ctrl_err = des - est
    est_err = est_cal - act_interp

    plt.figure(figsize=(16, 4))
    plt.subplot(131)
    plt.plot(des[:, 0], des[:, 1], 'r.', markersize=1)
    plt.plot(est_cal[:, 0], est_cal[:, 1], 'b:', markersize=1)
    plt.plot(act_interp[:, 0], act_interp[:, 1], 'k.', markersize=1)
    w, h = 2.9475, 2.2325
    # plt.plot([w, w, 0, 0, w], [0, h, h, 0, 0], 'saddlebrown')
    inds = [0,1,2,3,0]
    print(frame)
    plt.plot(frame[inds, 0], frame[inds, 1], 'saddlebrown')
    plt.legend(("Setpoint", "Estimated Position", "Ground Truth Position"), loc='upper right')
    plt.title('X vs Y (m)')
    plt.xlabel('x (m)')
    plt.xlabel('y (m)')
    plt.grid()
    plt.axis('equal')
    plt.subplot(132)
    plt.plot(t, ctrl_err[:, 0], 'r-')
    plt.plot(t, ctrl_err[:, 1], 'g-')
    plt.legend(('x', 'y'), loc='upper left')
    plt.title('Controller Tracking Error (m)')
    plt.xlabel('time (s)')
    plt.ylabel('Error (m)')
    plt.grid()
    plt.subplot(133)
    plt.plot(t, est_err[:, 0], 'r-')
    plt.plot(t, est_err[:, 1], 'g-')
    # plt.plot(t, est_cal[:, 0], 'r.')
    # plt.plot(t_act, act[:, 0], 'g.')
    plt.legend(('x', 'y'), loc='upper left')
    plt.title('Estimation Error (m)')
    plt.xlabel('time (s)')
    plt.ylabel('Error (m)')
    plt.grid()
    plt.suptitle(name)
    plt.savefig('_' + name + '_align-' + align_style + '.eps', format='eps')
    plt.savefig('_' + name + '_align-' + align_style + '.png', transparent=True, dpi=600)

    plt.figure()
    act_rot = act_rot / np.pi * 180
    plt.plot(t_act, act_rot[:, 0], 'r', markersize=1, linewidth=0.4)
    plt.plot(t_act, act_rot[:, 1], 'g', markersize=1, linewidth=0.4)
    plt.plot(t_act, act_rot[:, 2], 'b', markersize=1, linewidth=0.4)
    # plt.plot(t_act, act)
    plt.legend(('x', 'y', 'z'))
    plt.xlabel('time (s)')
    plt.ylabel('Angle (deg)')
    plt.grid(True)
    plt.title('End Effector Orientation')

    plt.figure()
    vel = np.diff(des, axis=0)
    print(vel[:10])
    plt.plot(np.linalg.norm(np.diff(des, axis=0), axis=1) / 0.01, '-')

    print("RMS Tracking Error:", np.sqrt(np.nanmean(np.square(ctrl_err))))
    print("Max Tracking Error:", np.nanmax(np.abs(ctrl_err)))
    print("RMS Estimation Error:", np.sqrt(np.nanmean(np.square(est_err))))
    print("Max Estimation Error:", np.nanmax(np.abs(est_err)))
    print("RMS Angle Deviation:", np.sqrt(np.nanmean(np.square(act_rot), axis=0)))
    print("Max Angle Deviation:", np.nanmax(np.abs(act_rot), axis=0))

    scipy.io.savemat(
        name + '.mat',
        {
            't': t,  # time
            'des': des,  # desired / setpoint position
            'est': est,  # estimated position
            'est_cal': est_cal,  # estimated position (offline calibrated)
            'gt': act_interp,  # mocap ground truth positions (already time-aligned)
            'est_err': est_err,  # estimation error = est_cal - gt
            'ctrl_err': ctrl_err,  # controller tracking error = des - est
            'act_rot': act_rot_interp,  # end effector rotation
            'frame': frame  # locations of the 4 pulleys (optional, for drawing cable robot outline)
        })

    plt.show()

if __name__ == '__main__':
    main('15_concentric_diamonds2_2mps_20mps2')
    # main('rot_x-y-z_(y=gravity)')
