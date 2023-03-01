This code will be ugly, but it's just a first-pass test to see if closed-loop control on Teensy @100Hz with 4 cables will work well

Note: gains are dependent upon control frequency.
@200Hz: Kp = 2000, Ki = 1000, Kd = 500, lpf (vel) = 0.05
@100Hz: Kp = 1000, Ki = 500, Kd = 250, lpf (vel) = 0.10