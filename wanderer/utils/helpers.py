import numpy as np


# half angle of the view area
VIEW_ANGLE = 10/180*np.pi


def polar2cart(angles, dists):
    x = dists * np.cos(angles)
    y = dists * np.sin(angles)
    return x, y


def cart2polar(x, y):
    d = np.sqrt(x**2 + y**2)
    a = np.arctan2(y, x)
    return a, d


def transform(x, y, t, rot):
    x2 = t[0] + np.cos(rot) * x + np.sin(rot) * y
    y2 = t[1] + np.sin(rot) * x + np.cos(rot) * y
    return x2, y2


def transform_polar(a, d, t, rot):
    x2 = t[0] + np.cos(a + rot) * d
    y2 = t[1] + np.sin(a + rot) * d
    return x2, y2


def predict_scan_points(a, d, t, rot):
    x, y = transform_polar(a, d, t, rot)
    a2, d2 = cart2polar(x, y)
    return a2, d2


def select_front_distances(a, d):
    idx = np.abs(a) <= VIEW_ANGLE
    return d[idx]


def main():
    x = np.ones(210)
    y = np.arange(10, -11, -0.1)
    a, d = cart2polar(x, y)
    # print(a)
    # print(d)

    # print(VIEW_ANGLE)
    print(select_front_distances(a, d))

    dt = 1
    movement = np.array([0, 0])
    rot = 20/180*np.pi

    a2, d2 = predict_scan_points(a, d, -movement, -rot)
    print('Predicted:')
    # print(a2)
    # print(d2)
    print(select_front_distances(a2, d2))


if __name__ == '__main__':
    main()
