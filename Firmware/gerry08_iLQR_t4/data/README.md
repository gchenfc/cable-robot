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
