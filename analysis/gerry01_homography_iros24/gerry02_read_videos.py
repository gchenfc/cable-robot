import argparse
import numpy as np
import cv2
import matplotlib.pyplot as plt
import tqdm as tqdm
from pathlib import Path
from pupil_apriltags import Detector


def main(video_file=Path('dropbox') / 'A.mov', EVERY_NTH_FRAME=10):
    detector = Detector(families="tagStandard41h12",
                        nthreads=1,
                        quad_decimate=1.0,
                        quad_sigma=0.0,
                        refine_edges=1,
                        decode_sharpening=0.25,
                        debug=0)

    # Test
    # Read first frame from video
    cap = cv2.VideoCapture((video_file).as_posix())
    success, image = cap.read()
    assert success, "Error reading video"
    cap.release()

    # Apriltag detection
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray)
    print(f'Found {len(detections)} tags in the first frame.')
    for detection in detections:
        print(detection.tag_id, detection.tag_family, detection.corners,
              detection.center)

    # Now read more frames from video
    all_detections = {}

    cap = cv2.VideoCapture((video_file).as_posix())
    total_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    with tqdm.tqdm(total=total_frames) as pbar:
        for i in range(int(total_frames)):
            success, image = cap.read()
            if not success:
                print(f'Error reading frame {i}')
                break
            if i % EVERY_NTH_FRAME == 0:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                detections = detector.detect(gray)
                all_detections[i] = detections
                pbar.set_description(
                    f'Frame {i}: Found {len(detections)} tags')
            if i % 1000 == 0:
                # Save intermediate results
                name = video_file.name
                np.savez((
                    video_file.parent / 'data' /
                    f'{video_file.stem}_detections_every.{EVERY_NTH_FRAME}.npz'
                ).as_posix(),
                         all_detections=all_detections)
            pbar.update(1)
    cap.release()
    np.savez((
        video_file.parent / 'data' /
        f'{video_file.stem}_detections_every{EVERY_NTH_FRAME}.npz').as_posix(),
             all_detections=all_detections)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('video_file', type=Path)
    parser.add_argument('--every_nth_frame', type=int, default=10)
    args = parser.parse_args()
    print('parsed args:', args)
    if input('ok?') == 'y':
        main(args.video_file, args.every_nth_frame)
