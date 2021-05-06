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
