"""
continuous_recalibration.py

This script is used to continuously recalibrate the cable robot using AprilTag detections as
position measurements.

It takes over SerialUSB2 (the fast one) and sends updates by:

while True:
    Read the cable robot cable lengths
    Read the AprilTag detection position
    Compute the theoretically correct cable length
    Update the constant parameter in the cable length correction parameters to the robot

Recall that the cable length correction is computed as:
    l_corrected = params[0] * np.square(l_raw) + l_raw * params[1] + params[2]

We will just update params[2] every update, because it's hard to guarantee that the other 2 are
observable.
"""

import argparse
import time
import socket
import read_tags
import util
import numpy as np
import scipy.optimize
from read_tags import Detector

import sys

sys.path.append('..')
from gerry09.communicate import CableRobot

Timestamp = float
Rotation = np.ndarray[3, 3]
Translation = np.ndarray[3]


class ContinuousRecalibration:
    # CONSTANTS
    W_in, H_in = 227.25, 12 * 12 + 0.75 - 3
    W, H = W_in * 0.0254, H_in * 0.0254
    DEFAULT_CAMERA_PARAMS = {
        'tag_size': 0.26416,  # meters (13/10*8 inches)
        'f': 2900 // 2,  # focal length in pixels, adjust till depth is about 11.03m
        'cx': 4032 // 2 // 2,  # camera center in pixels
        'cy': 3024 // 2 // 2,  # camera center in pixels
    }  # second divide by 2 is because iOS app downsamples by 2
    TAG_LOCATIONS_IN = {
        0: (18.75 + W_in, 34.25),  #
        2: (-22.25, 76.5),
        3: (-26.75, 34.0)
    }
    TAG_LOCATIONS = {k: (v[0] * 0.0254, v[1] * 0.0254) for k, v in TAG_LOCATIONS_IN.items()}
    EE_LOCATION_IN = (18.5, 2)
    EE_LOCATION = (EE_LOCATION_IN[0] * 0.0254, EE_LOCATION_IN[1] * 0.0254)
    W_EE, H_EE = 20.5 * 0.0254, 15.75 * 0.0254
    ANCHOR_POINTS = np.array([[W - W_EE, 0], [W - W_EE, H - H_EE], [0, H - H_EE], [0, 0]])

    # INITIALIZATION
    def __init__(self,
                 initial_lparams: np.ndarray[3, 4],
                 PORT='/dev/tty.usbmodem103568503',
                 lpf_alpha=0.01,
                 slew_rate_m_per_detection=0.0001,
                 send_interval_s=1):
        self.params = initial_lparams.copy()
        self.robot = CableRobot(print_raw=True, write_timeout=None, initial_msg='d10,10', port=PORT)
        self.apriltag = Detector(self.DEFAULT_CAMERA_PARAMS, timeout=0.1)
        self.lpf_alpha = lpf_alpha
        self.slew_rate_m_per_detection = slew_rate_m_per_detection
        self.last_t = None
        self.send_interval_s = send_interval_s
        self.last_send = time.time()

    def __enter__(self):
        self.robot.__enter__()
        self.apriltag.__enter__()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.robot.send('d10,100')
        self.apriltag.__exit__(exc_type, exc_val, exc_tb)
        self.robot.__exit__(exc_type, exc_val, exc_tb)

    # MASTER LOOP
    def update(self, dry_run=True):
        self.read_robot()
        try:
            detection = self.read_one_tag(timeout=0.1)
        except (TimeoutError, socket.timeout):
            return
        self.last_t = detection[0]
        self.read_robot()
        params = self.calibrate(*detection)
        slew_rate = self.slew_rate_m_per_detection / self.lpf_alpha
        self.params += self.lpf_alpha * np.clip(params - self.params, -slew_rate, slew_rate)
        if (self.last_send + self.send_interval_s) < time.time():
            self.last_send = time.time()
            self.send_params_to_robot(dry_run=dry_run)

    # IO
    def read_robot(self):
        self.robot.update()
        self.robot.all_data = self.robot.all_data[-100:]

    def send_params_to_robot(self, dry_run=True):
        # self.robot.send('c' + ','.join(str(p) for p in self.params.flatten()))
        s2 = 'c45,' + ','.join(map(str, self.params.T.flatten()))
        if dry_run:
            print(s2)
        else:
            self.robot.send(s2)
            # print(s2)

    def read_one_tag(self, timeout=1):
        tstart = time.time()
        while tstart + timeout > time.time():
            (t_cap, t), detections = self.apriltag.detect()
            t_now = time.time()
            anchor_locs = {
                det.tag_id: det.Rt()[1] for det in detections if det.tag_id in self.TAG_LOCATIONS
            }
            try:
                ee_index = [det.tag_id for det in detections].index(4)
            except ValueError:
                continue
            if ee_index >= 0:
                ee_loc = detections[ee_index].Rt()[1]
                break
        else:
            raise TimeoutError('No tag found in timeout period')
        return (t_now - t + t_cap), anchor_locs, ee_loc
        # return t_cap, anchor_locs, ee_loc

    # CALIBRATION FUNCTIONS
    @classmethod
    def canonicalize_anchors(cls, anchor_locs: dict[int,
                                                    Translation]) -> tuple[Rotation, Translation]:
        return util.compute_Rt_from_anchors(cls.TAG_LOCATIONS, anchor_locs, verbosity=0)

    @classmethod
    def canonicalize_ee(cls, ee_loc: Translation, anchor_locs: dict[int,
                                                                    Translation]) -> Translation:
        Rt = cls.canonicalize_anchors(anchor_locs)
        return util.apply(Rt, ee_loc.reshape(1, 3)).flatten()

    def nearest_robot_ls(self, t: Timestamp) -> np.ndarray[4]:
        ts = np.array([d['timestamp'] for d in self.robot.all_data])
        # Find the index of the nearest ts
        i = np.argmin(np.abs(ts - t))
        return [m.length for m in self.robot.all_data[i]['motors']]

    def nearest_robot_ls_interp(self, t: Timestamp) -> np.ndarray[4]:
        """Interpolate between the 2 nearest times"""
        ts = np.array([d['timestamp'] for d in self.robot.all_data])
        # Find the indices of the nearest times
        i = np.argmin(np.abs(ts - t))
        if ts[i] > t:
            i -= 1
        i = np.clip(i, 0, len(ts) - 2)
        # Interpolate
        t1, t2 = ts[i:i + 2]
        l1 = np.array([m.length for m in self.robot.all_data[i]['motors']])
        l2 = np.array([m.length for m in self.robot.all_data[i + 1]['motors']])
        return l1 + np.clip((t - t1) / (t2 - t1), 0, 1) * (l2 - l1)

    def calibrate(self, t: Timestamp, anchor_locs: dict[int, Translation], ee_loc: Translation):
        raw_ls = self.nearest_robot_ls_interp(t)
        l_meas = util.l_correction(raw_ls, self.params)
        ee_loc = self.canonicalize_ee(ee_loc, anchor_locs)
        ee_loc2 = ee_loc[:2] - self.EE_LOCATION
        l_true = np.sqrt(np.sum(np.square(ee_loc2 - self.ANCHOR_POINTS), axis=1))

        new_params = self.params.copy()
        new_params[2] += l_true - l_meas
        # print('lengths measured vs true:', l_meas, l_true)
        # print('params old vs new:')
        # print(self.params)
        # print(new_params)
        # print('updated l measurement:')
        # print(util.l_correction(raw_ls, new_params))
        return new_params


def main():
    initial_lparams = np.array(
        [[-5.39622604e-03, -7.55211558e-04, -6.64543613e-03, -1.23351185e-02],
         [4.89951909e-01, 2.22615412e-01, 2.81593933e-01, 5.03432217e-01],
         [2.57621490e+00, 4.13689341e+00, 4.23045204e+00, 2.70388820e+00]])
    # with ContinuousRecalibration(initial_lparams=initial_lparams, lpf_alpha=0.01) as cr:
    with ContinuousRecalibration(initial_lparams=initial_lparams,
                                 send_interval_s=0,
                                 lpf_alpha=0.1,
                                 slew_rate_m_per_detection=0.0001) as cr:
        while True:
            cr.update(dry_run=False)



if __name__ == '__main__':
    main()
