import numpy as np
import matplotlib.pyplot as plt


def main():
    data = np.loadtxt('circle1_gain75_off5_low-01.txt',
                      comments="S",
                      usecols=(1, 2, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15, 17, 18, 19, 20, 22, 23, 24,
                               25))
    data = data[data[:, 2] != 0, :]

    t = np.arange(0, data.shape[0] / 100, 0.01)
    est = data[:, :2]
    des = data[:, 2:4]
    errors = data[:, 4::4]
    states = data[:, 5::4]
    lens = data[:, 6::4]
    vels = data[:, 7::4]

    err = des - est

    plt.figure(figsize=(10, 4))
    plt.subplot(121)
    plt.plot(est[:, 0], est[:, 1], 'k-')
    plt.plot(des[:, 0], des[:, 1], 'b-')
    plt.grid()
    plt.axis('equal')
    plt.subplot(122)
    plt.plot(t, err[:, 0], 'k-')
    plt.plot(t, err[:, 1], 'b-')
    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()
