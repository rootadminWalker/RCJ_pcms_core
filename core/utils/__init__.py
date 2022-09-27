import numpy as np


def depth_avoid_zeropoints(point, depth_image, limit=None):
    if limit is None:
        limit = depth_image.shape[0]

    x, y = point
    for each in range(1, limit):
        up = y - each
        down = y + each
        left_x = x - each
        right_x = x + each

        top = depth_image[up:up + 1, left_x:right_x + 1]
        left = depth_image[up:up + 1, left_x:left_x + 1]
        bottom = depth_image[down:down + 1, left_x:left_x + 1]
        right = depth_image[up:down + 1, right_x:right_x + 1]

        for block in [top, left, bottom, right]:
            nonzero = block[np.nonzero(block)]
            if nonzero.shape[0] > 0:
                distance = nonzero[0]
                return distance
