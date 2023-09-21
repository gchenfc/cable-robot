from scipy.spatial.transform import Rotation
import scipy.optimize
import scipy.linalg
from scipy.interpolate import splrep, BSpline
import numpy as np


def Rt_from_params(params):
    R0 = Rotation.from_euler('xyz', [np.pi, 0, 0])
    R = R0 * Rotation.from_rotvec(params[:3])
    t = params[3:6]
    return R.as_matrix() * params[6], t


def apply(Rt, xyz):
    R, t = Rt
    return (R @ xyz.T + t[:, None]).T


def compute_Rt_from_anchors(TAG_LOCATIONS: dict[int, np.ndarray],
                            mean_anchors: dict[int, np.ndarray], verbosity=2) -> tuple[np.ndarray, np.ndarray]:
    """Computes the Rt transformation from the mean_anchors to the TAG_LOCATIONS."""

    def err(dx):
        Rt = Rt_from_params(dx)
        err = np.zeros((len(TAG_LOCATIONS), 3))
        for i, (tag_id, loc) in enumerate(TAG_LOCATIONS.items()):
            err[i] = apply(Rt, mean_anchors[tag_id].reshape(1, 3)) - np.array([loc[0], loc[1], 0])
        return err.flatten()

    # Could use ICP closed-form equation but too lazy.
    # I bet normal equation ends up being same anyway.
    result = scipy.optimize.least_squares(err, np.ones(7), verbose=verbosity, method='lm')
    if verbosity > 0:
        print(result.success, result.message, result.x)

    Rt = Rt_from_params(result.x)
    return Rt


def align_timestamps(t1, v1, t2):
    """Smooths+interpolates t1,v1 and queries at t2.  Returns v1(t2)."""
    spline1 = [splrep(t1, v1_, s=10) for v1_ in v1.T]
    return np.array([BSpline(*s)(t2) for s in spline1]).T


def l_correction(ls, params: np.ndarray[3, 4]):
    assert params.shape == (3, 4)
    return params[0] * np.square(ls) + ls * params[1] + params[2]

class CableLengthCalibrationWithGt:

    def __init__(self, ls, xs, mount_points):
        self.ls = ls
        self.xs = xs
        self.MOUNT_POINTS = mount_points

    # Helper functions
    def l_corr(self, params, ls=None):
        if ls is None:
            ls = self.ls
        return l_correction(ls, params.reshape(-1, 4))

    def ik(self):
        xs = self.xs.reshape(-1, 2, 1)
        mountPoints = self.MOUNT_POINTS.reshape(1, 2, 4)
        return np.sqrt(np.sum(np.square(xs - mountPoints), axis=1))

    def err(self, params):
        return (self.l_corr(params) - self.ik()).flatten()

    # Actual Calibrate Function Call
    def calibrate(self, init_params=None):
        if init_params is None:
            init_params = np.array([0, 0, 0, 0, 0.5, 0.5, 0.5, 0.5, *(-self.ls.mean(axis=0) + 1.5)])
        result = scipy.optimize.least_squares(self.err, init_params, verbose=2, method='lm')
        self.lparams = result.x.reshape(-1, 4)
        return result

    def compute_x_estimates(self, subsample=10):
        l_act = self.l_corr(self.lparams, self.ls[::subsample])

        def err2(xs):
            xs = xs.reshape(-1, 2, 1)
            mountPoints = self.MOUNT_POINTS.reshape(1, 2, 4)
            lhats = np.sqrt(np.sum(np.square(xs - mountPoints), axis=1))
            return (lhats - l_act).flatten()

        def jac(xs):
            xs = xs.reshape(-1, 2, 1)
            mountPoints = self.MOUNT_POINTS.reshape(1, 2, 4)
            dx = xs - mountPoints
            lhats = np.sqrt(np.sum(np.square(dx), axis=1))
            ret = dx / lhats[:, None, :]
            # Now I need to use diagonal and stuff to make jac be (n*4, n*2)
            return scipy.linalg.block_diag(*ret).T

        init_params = self.xs[::subsample].flatten()
        result = scipy.optimize.least_squares(err2,
                                              init_params,
                                              jac=jac,
                                              verbose=2,
                                              method='lm')
        if not result.success:
            raise RuntimeError(result.message)
        return result.x.reshape(-1, 2)
