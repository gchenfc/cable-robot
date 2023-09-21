"""
read_tags.py

Python class reads AprilTag iOS UDP messages and parses them into a list of detections.
Computes rotation & translation matrices as well using cv2 solvePnP.

See also: https://github.com/gchenfc/apriltags-ios in which I make some modifications to:
    * Manually adjust exposure, ISO, etc.
    * Send the timestamp of image acquisition over UDP (in addition to timestamp after detections,
        to subtract the offset between computer & phone time syncing)

Usage:
    python read_tags.py

or:
    with Detector(PARAMS) as detector:
        (t_capture, t_send), detections = detector.detect()

    PARAMS is an optional dictionary of camera parameters:
        DEFAULT_CAMERA_PARAMS = {
            'tag_size': 0.1905,  # meters (7.5 inches)
            'f': 3072 // 2 // 2,  # focal length in pixels, hand-measured so probably not precise
            'cx': 4032 // 2 // 2,  # camera center in pixels
            'cy': 3024 // 2 // 2,  # camera center in pixels
        }  # second divide by 2 is because iOS app downsamples by 2
"""
import socket
import struct
import itertools
import dataclasses
import numpy as np
import cv2
import time
import pickle


def get_local_ip():
    try:
        # This creates a new socket and connects to an external server (in this case, Google's public DNS server).
        # We don't actually send any data, we just use this to determine the most appropriate local IP address.
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except Exception:
        return None


UDP_IP = get_local_ip()
UDP_PORT = 7709


@dataclasses.dataclass
class Detection:
    tag_family: str = None
    "The family of the tag."
    tag_id: int = None
    "The decoded ID of the tag."
    hamming: int = None
    """How many error bits were corrected?"""
    decision_margin = None
    """A measure of the quality of the binary decoding process"""
    homography: list[float] = None
    """
        The 3x3 homography matrix describing the projection from an "ideal" tag (with
        corners at (-1,1), (1,1), (1,-1), and (-1, -1)) to pixels in the image.
        """
    center: tuple[float, float] = None
    "The center of the detection in image pixel coordinates."
    corners: list[list[float]] = None
    """The corners of the tag in image pixel coordinates. These always wrap counter-
        clock wise around the tag."""
    pose_R = None
    r"\* Rotation matrix of the pose estimate - cRo is the rot from object frame to camera frame"
    pose_t = None
    r"\* Translation of the pose estimate - cPo is the trans from object frame to camera frame"
    pose_err = None
    r"\* Object-space error of the estimation."

    @property
    def R(self):
        return self.pose_R

    @property
    def t(self):
        return self.pose_t

    def CameraMatrix(fx, fy, cx, cy):
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)

    CORNERS = np.array([[1, 1, 0], [1, 0, 0], [0, 0, 0], [0, 1, 0]], dtype=float)

    def Rt(self, camera_matrix=None, tag_size=1, f=None, cx=None, cy=None):
        """
        iPhone coordinate frame:
            charging port is +x
            volume up is +y
            back of phone is +z
        """
        if self.pose_R is None or self.pose_t is None:
            if camera_matrix is None:
                if f is None or cx is None or cy is None:
                    raise Exception('camera_matrix or f, cx, cy must be provided')
                camera_matrix = Detection.CameraMatrix(f, f, cx, cy)
            ok, r, self.pose_t = cv2.solvePnP(Detection.CORNERS * tag_size, self.corners,
                                              camera_matrix, None)
            self.pose_R, _ = cv2.Rodrigues(r)
        return self.pose_R, self.pose_t

    def __repr__(self) -> str:
        c = self.center
        p = self.corners
        H = self.homography
        return '\n'.join([
            f'  id={self.tag_id:2d}, err={self.hamming}, '
            f'family={self.tag_family:2d} at ({c[0]:8.2f}, {c[1]:8.2f})',
            f'    corners: ' + np.array2string(p, prefix=' ' * 13),
            f'    H: ' +
            np.array2string(H, formatter={'float_kind': lambda x: "%7.2f" % x}, prefix=' ' * 7),
        ])


class DataInputStream:

    def __init__(self, packet):
        self.packet = iter(packet)

    def read_int(self):
        return int.from_bytes(bytes([next(self.packet) for i in range(4)]), byteorder='big')

    def read_float(self):
        return struct.unpack('>f', bytearray(itertools.islice(self.packet, 4)))[0]


def process_packet(packet):
    ins = DataInputStream(packet)
    MAGIC1 = ins.read_int()
    MAGIC2 = ins.read_int()
    assert MAGIC1 == 0x41505249
    assert MAGIC2 == 0x4c544147

    version = ins.read_int()
    assert (version == 0x00010002) or (version == 0x00010102), f"version mismatch: {version:x}"

    ndets = ins.read_int()
    utime = ((ins.read_int()) << 32) + ins.read_int()
    if (version == 0x00010102):
        utime_cap = ((ins.read_int()) << 32) + ins.read_int()
    else:
        utime_cap = 0

    detections = []
    for i in range(ndets):
        id = ins.read_int()
        hamming = ins.read_int()
        ncodes = ins.read_int()  # This is a proxy for the tag family

        c = np.array([ins.read_float(), ins.read_float()])

        p = np.zeros((4, 2))
        for j in range(4):
            p[j][0] = ins.read_float()
            p[j][1] = ins.read_float()

        H = np.zeros(9)
        for j in range(9):
            H[j] = ins.read_float()
        H = H.reshape((3, 3))

        detections.append(
            Detection(tag_family=ncodes,
                      tag_id=id,
                      hamming=hamming,
                      center=c,
                      corners=p,
                      homography=H))
    assert next(ins.packet, None) is None, 'packet not fully consumed'
    return (utime_cap / 1e6, utime / 1e6), detections


DEFAULT_CAMERA_PARAMS = {
    'tag_size': 0.1905,  # meters (7.5 inches)
    'f': 3072 // 2 // 2,  # focal length in pixels, hand-measured so probably not precise
    'cx': 4032 // 2 // 2,  # camera center in pixels
    'cy': 3024 // 2 // 2,  # camera center in pixels
}  # second divide by 2 is because iOS app downsamples by 2


class Detector(socket.socket):

    def __init__(self, camera_params=DEFAULT_CAMERA_PARAMS, timeout=1.0):
        self.camera_params = camera_params
        super().__init__(socket.AF_INET, socket.SOCK_DGRAM)
        self.settimeout(timeout)
        self.bind((UDP_IP, UDP_PORT))
        print('In the Apriltag iOS app, set UDP transmict Addr to:', UDP_IP)

    def detect(self):
        t, detections = process_packet(self.recv(4096))
        for detection in detections:
            detection.Rt(**self.camera_params)
        return t, detections


def main():
    # CAMERA PARAMETERS
    camera_params = DEFAULT_CAMERA_PARAMS

    # # for testing
    # data = pickle.load(open("test_data.p", "rb"))
    # t, detections = process_packet(data)
    # print(f'{t:.3f}: {len(detections)} tags')
    # for detect in detections:
    #     print(detect)
    #     R, t = detect.Rt(**camera_params)
    #     print('    R = ' + np.array2string(R, prefix=' ' * 8))
    #     print('    t = ' + np.array2string(t, prefix=' ' * 8))

    # Run as a loop
    with Detector() as detector:
        while True:
            (t_cap, t), detections = detector.detect()
            t_now = time.time()
            print(
                f'{t_cap:.3f} (sync: {t_now - t:.3f}s, process_time: {t - t_cap:.3f}s): {len(detections)} tags'
            )
            for detect in detections:
                print(detect)
                print('    R = ' + np.array2string(detect.R, prefix=' ' * 8))
                print('    t = ' + np.array2string(detect.t, prefix=' ' * 8))


if __name__ == '__main__':
    main()
