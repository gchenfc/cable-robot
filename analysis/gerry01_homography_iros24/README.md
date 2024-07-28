## IROS 2024 Homography Experiment

This experiment is to justify the decision to use homographies to warp the trajectory in the task space, since pure joint-space calibration is insufficient.

The idea is to first to a normal proprioceptive calibration, then "touch" some predefined set of points in the task space to warp the task-space trajectories to match the correct canvas.

We want to compare errors on the same trajectory with 3 different calibration strategies:
A. Joint-space calibration only (what I normally do)
B. Joint-space w/ task-space keypoints calibration (do a joint optimization)
C. Joint-space first, then task-space after.
D. Joint-space first, then task-space after, multiple homographies.

See also:
* gerry00a_calibrate_air.ipynb
* gerry07_corner_homography_calibration.ipynb


## Collect data

You can collect data using:
```bash
screen -L /dev/tty.usbmodem100994303 && mv screenlog.0 test_$(date +"%Y-%m-%d_%Hh%Mm%Ss").log
```


# Experimental permutations notes 

### Original Calibration sent to robot:
[[3.033645 3.033645 0.       0.      ]
 [0.       2.359945 2.359945 0.      ]]
[[-9.14903237e-03 -6.34900140e-03 -1.20265249e-03  1.38619546e-04]
 [ 9.54793036e-01  9.48467699e-01  9.53947223e-01  9.96290971e-01]
 [ 1.49165973e+00  1.97623330e+00  2.07548955e+00  1.62627886e+00]]
----------------------------------------
c54,3.033645,0.0,3.033645,2.3599449999999997,0.0,2.3599449999999997,0.0,0.0
c44,-0.009149032369879625,0.954793035741132,1.49165973317845,-0.00634900139960473,0.9484676985408691,1.9762333004092443,-0.001202652491037734,0.9539472234851456,2.07548954544515,0.0001386195460796259,0.9962909711464595,1.626278863872799
----------------------------------------
c54,3.033645,0.0,3.033645,2.3599449999999997,0.0,2.3599449999999997,0.0,0.0;c44,-0.009149032369879625,0.954793035741132,1.49165973317845,-0.00634900139960473,0.9484676985408691,1.9762333004092443,-0.001202652491037734,0.9539472234851456,2.07548954544515,0.0001386195460796259,0.9962909711464595,1.626278863872799
Setting winch 0 mount point to {3.034, 0.000}
Setting winch 1 mount point to {3.034, 2.360}
Setting winch 2 mount point to {0.000, 2.360}
Setting winch 3 mount point to {0.000, 0.000}
Setting winch 0 length correction parameters to {-0.009, 0.955, 1.492}
Setting winch 1 length correction parameters to {-0.006, 0.948, 1.976}
Setting winch 2 length correction parameters to {-0.001, 0.954, 2.075}
Setting winch 3 length correction parameters to {0.000, 0.996, 1.626}

But then it got lost due to computer reboot...
#### New Calibration for B.
This wasn't actually sent to the robot, but it is what would have been sent to the robot if we didn't do the corner calibration.  That is, since we don't have the source ls and xys data anymore that was used to generate calibration A, instead we needed to collect new data.  Just for reference, that new data would have generated the following parameters to send to the robot:

[[3.033645 3.033645 0.       0.      ]
 [0.       2.359945 2.359945 0.      ]]
[[-6.08975030e-03 -3.95277842e-03 -1.78876867e-03 -3.30985420e-03]
 [ 9.47853679e-01  9.48203841e-01  9.54278689e-01  9.96805381e-01]
 [ 1.51538932e+00  1.98766908e+00  2.08443000e+00  1.64216570e+00]]
----------------------------------------
c54,3.033645,0.0,3.033645,2.3599449999999997,0.0,2.3599449999999997,0.0,0.0
c44,-0.006089750297821808,0.9478536788451588,1.5153893197865411,-0.003952778415774346,0.9482038412910407,1.9876690840157516,-0.0017887686711308532,0.9542786892923691,2.084430000096837,-0.0033098542018340903,0.9968053807425477,1.6421656985718156
----------------------------------------
c54,3.033645,0.0,3.033645,2.3599449999999997,0.0,2.3599449999999997,0.0,0.0;c44,-0.006089750297821808,0.9478536788451588,1.5153893197865411,-0.003952778415774346,0.9482038412910407,1.9876690840157516,-0.0017887686711308532,0.9542786892923691,2.084430000096837,-0.0033098542018340903,0.9968053807425477,1.6421656985718156
Setting winch 0 mount point to {3.034, 0.000}
Setting winch 1 mount point to {3.034, 2.360}
Setting winch 2 mount point to {0.000, 2.360}
Setting winch 3 mount point to {0.000, 0.000}
Setting winch 0 length correction parameters to {-0.006, 0.948, 1.515}
Setting winch 1 length correction parameters to {-0.004, 0.948, 1.988}
Setting winch 2 length correction parameters to {-0.002, 0.954, 2.084}
Setting winch 3 length correction parameters to {-0.003, 0.997, 1.642}

### B. joint calibration
Note: optimized with "carriage center" coordinates, so in gcode_webpage, need to offset by -baseTcdprrest:
array([0.0935, 0.061 ])

I ALREADY CORRECTED FOR THIS DURING DATA COLLECTION SO WE SHOULDN'T NEED TO CORRECT FOR IT AGAIN

[[3.033645 3.033645 0.       0.      ]
 [0.       2.359945 2.359945 0.      ]]
[[-6.08975030e-03 -3.95277842e-03 -1.78876867e-03 -3.30985420e-03]
 [ 9.47853679e-01  9.48203841e-01  9.54278689e-01  9.96805381e-01]
 [ 1.51538932e+00  1.98766908e+00  2.08443000e+00  1.64216570e+00]]
----------------------------------------
c54,3.033645,0.0,3.033645,2.3599449999999997,0.0,2.3599449999999997,0.0,0.0
c44,-0.006089750297821808,0.9478536788451588,1.5153893197865411,-0.003952778415774346,0.9482038412910407,1.9876690840157516,-0.0017887686711308532,0.9542786892923691,2.084430000096837,-0.0033098542018340903,0.9968053807425477,1.6421656985718156
----------------------------------------
c54,3.033645,0.0,3.033645,2.3599449999999997,0.0,2.3599449999999997,0.0,0.0;c44,-0.006089750297821808,0.9478536788451588,1.5153893197865411,-0.003952778415774346,0.9482038412910407,1.9876690840157516,-0.0017887686711308532,0.9542786892923691,2.084430000096837,-0.0033098542018340903,0.9968053807425477,1.6421656985718156
Setting winch 0 mount point to {3.034, 0.000}
Setting winch 1 mount point to {3.034, 2.360}
Setting winch 2 mount point to {0.000, 2.360}
Setting winch 3 mount point to {0.000, 0.000}
Setting winch 0 length correction parameters to {-0.006, 0.948, 1.515}
Setting winch 1 length correction parameters to {-0.004, 0.948, 1.988}
Setting winch 2 length correction parameters to {-0.002, 0.954, 2.084}
Setting winch 3 length correction parameters to {-0.003, 0.997, 1.642}


### C. 1 Homography
[[0.5237989279341662, -0.01518768038512393, 0.1953645980517818], [-0.021674168983994806, 0.5235055693660097, 0.18547654537906982], [-0.017189510876584643, -0.01243733068295738, 0.6147008420075405]]

### D. Homography grid
A 3x3 grid of points was measured to generate a 2x2 grid of homography squares.
The homography was pre-computed in zigzag_multigrid.nc/.npy.

##
Raw data & videos can be found at https://www.dropbox.com/scl/fo/l4j7drvby8tywquk1wfwr/h?rlkey=i4s7mduecolj6zpip5660g2dj&dl=0
