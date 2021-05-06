# Data

## ATL_0.txt
ATL logo, with `ATL_controller_1e2.h`.

** attention: the spray can was empty!!!  It was the maroon "Montana BLACK" color.

Controller parameters:
| | |
|-|-|
| Q / R ratio | 1e2 |
| End-effector mass | 0.5 kg |
| Motor inertia | 9.26e-5 * 890 / 420   
|| = 1.96e-4 kg.m^2 |
| Friction | no friction |

Videos:
https://www.dropbox.com/sh/3pdi668omjyopq8/AAAZWnLFw1qKdHOouw-Wl0Qia?dl=0

## ATL_1.txt
Q/R ratio of 1e6 was way too powerful and error-ed out.

## ATL_2.txt
Attempt number 2 with Q/R ratio of 1e6 - still too powerful.
Video: https://www.dropbox.com/s/ymhe4rv08weq9zu/ATL_2.mov?dl=0

## ATL_3.txt
Attempt number 3 with Q/R ratio of 1e6.  This time I tried adding current limits, but that still didn't work.
Video: https://www.dropbox.com/s/z3avdaa9wqkob6h/ATL_3.mov?dl=0

## ATL_4.txt
Use `ATL_controller_1e2.h` again, but with a heavier spray can (full)
Same parameters as in ATL_0.txt, just with full spray can (white "KOBRA")
Video:
https://www.dropbox.com/s/5u2ohaypsewaw3d/ATL_4.mov?dl=0

## ATL_5.txt
Use `ATL_controller_1e2.h` again, but also with a battery pack.  Same parameters.
Qualitatively, I found that it was much more shaky, especially in the rotational direction, with normal vector in the z-axis (yaw).

The additional shakiness makes sense - if I don't account for friction properly, then it's going to be more shaky as the end-effector mass increases.

The rotating also makes sense, since the battery pack is mounted off-center.

Video: https://www.dropbox.com/s/gnuyepocstixagp/ATL_5.mov?dl=0

## ATL_6.txt
Use `ATL_controller_1e2.h` again, but put battery pack on the bottom of the end effector instead of the front.

As hypothesized, it is still jerky but not rotate-y.  Visually, I saw pretty much no rotation, though it's hard to tell on video.

Video: https://www.dropbox.com/s/85c0rozttg51xto/ATL_6.mov?dl=0

## ATL_7.txt

Use `ATL_controller_1e2_static.h`

Added static friction of 0.075Nm to the controller model.

Controller parameters:
| | |
|-|-|
| Q / R ratio | 1e2 |
| End-effector mass | 0.5 kg |
| Motor inertia | 9.26e-5 * 890 / 420   
|| = 1.96e-4 kg.m^2 |
| **Static Friction** | **0.075 Nm** |
| Viscous Friction | 0 |

(note: adding viscous friction caused the optimizer to fail)

<ins>Observations</ins>:
* The corners are certainly much sharper.  It was really easy to tell in person that direction changes were really snappy.  I bet this would show up much better during a lawnmower infill path.  Not sure if it will show up in the data or not, due to limited framerate of data collection (20Hz for telemetry and 30Hz video)
* The linear parts of the trajectory looked a little more shaky, but I think this showed up in the ff (gtsam open-loop prediction, see image below) trajectory as well, so that's expected.  Perhaps these correspond to times when the velocity of a cable changes during a straight-line and might be mitigated either with tuning or with a less sharp `sign` model (e.g. reduce "epsilon" in tanh).
  ![gtsam prediction](../trajectories/ATL_controller_1e2_static.png)

Video: https://www.dropbox.com/s/ml5hqma2lvh8svc/ATL_7.mov?dl=0

## ATL_8
Use `ATL_controller_1e0.h` - smaller Q/R ratio

Controller parameters:
| | |
|-|-|
| **Q / R ratio** | **1** |
| End-effector mass | 0.5 kg |
| Motor inertia | 9.26e-5 * 890 / 420   
|| = 1.96e-4 kg.m^2 |
| Friction | no friction |

In simulation this looked super cool, but in practice this totally failed.  I think this is because static friction wasn't accounted for and the gains were just too weak to compensate.

![gtsam prediction](../trajectories/ATL_controller_1e0.png)

Video: https://www.dropbox.com/s/8vfghn7nrmquq6k/ATL_8.mov?dl=0

## ATL_9

Use `ATL_controller_1e0_static.h` - add static friction

Controller parameters:
| | |
|-|-|
| **Q / R ratio** | **1** |
| End-effector mass | 0.5 kg |
| Motor inertia | 9.26e-5 * 890 / 420   
|| = 1.96e-4 kg.m^2 |
| **Static Friction** | **0.075 Nm** |
| Viscous Friction | 0 |

This neither looked cool in simulation nor in real life.
Also the optimizer actually didn't converge but I just let it be.

![gtsam prediction](../trajectories/ATL_controller_1e0_static.png)

Video: https://www.dropbox.com/s/cdn4df2vlavsa3l/ATL_9.mov?dl=0

## ATL_10

Going back to very first test, just to make sure it's repeatable over time.

Running with `ATL_controller_1e2.h`.

attention: the spray can is not empty (aka full) and has the battery pack on the bottom, so I guess technically this is more like test ATL_6.

Controller parameters:
| | |
|-|-|
| Q / R ratio | 1e2 |
| End-effector mass | 0.5 kg |
| Motor inertia | 9.26e-5 * 890 / 420   
|| = 1.96e-4 kg.m^2 |
| Friction | no friction |

Results:
* It worked pretty much just as well as ATL_6 from what I can tell.  Was a little shaky compared to ATL_0, as expected, due to extra mass.

## ATL_11

Ok this is really back to the first test: no spray can and no battery pack

Running with `ATL_controller_1e2.h`.

Results:
* It worked well - smoother than ATL_10, but still a _little_ bit jerky.  Hard to recall if ATL_0 was jerky at all

Video: https://www.dropbox.com/s/fu5udrhp3vvfyak/ATL_11.mp4?dl=0
