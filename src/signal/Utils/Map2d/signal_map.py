from numpy import array, ones

from Utils.Statistical import min_error
from Utils.Error import error_distance as ed
from Utils.Error import round_int64 as rd


def map2darray(y: list, x: list, data: list, gap: float = 0.5, error_function=min_error):
    # convert to ndarray
    y = array(y)
    x = array(x)

    # scale
    y_scale = y / gap
    x_scale = x / gap

    # round
    y_norm = rd(y_scale)
    x_norm = rd(x_scale)
    # print('y_norm', y_norm)
    # print('x_norm', x_norm)

    # natural
    y_N = y_norm - y_norm.min()
    x_N = x_norm - x_norm.min()
    # print('y_N', y_N)
    # print('x_N', x_N)

    # data struct
    data_set = {p: {} for p in set(zip(y_norm, x_norm))}
    # print(data_set)
    for yy, xx, dd in zip(y_scale, x_scale, data):
        # print(len(y_scale))
        # print(len(x_scale))
        # print(len(data))
        # print('rd(yy):', rd(yy))
        # print('rd(xx):', rd(xx))
        # print('ed(yy, xx):', ed(yy, xx))
        # print('dd:', dd)
        data_set[(rd(yy), rd(xx))][ed(yy, xx)] = dd
        

    # print('data_set:', data_set)

    # create 2d minimum power map
    sizeY = y_N.max() - y_N.min() + 1
    sizeX = x_N.max() - x_N.min() + 1
    map2d = -120 * ones((sizeY, sizeX))
    # print('map2d', map2d)

    for (yy, xx), dd in data_set.items():
        map2d[yy - y_norm.min()][xx - x_norm.min()] = error_function(dd)
        # print(map2d)
    return map2d
