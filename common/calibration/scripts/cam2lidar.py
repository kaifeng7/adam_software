#!/usr/bin/env python
# -*- utf-8 -*-

import numpy as np
import ros
import tf


def main():
    rigid_tf = np.array(
        [[0.997804, -0.00260936, -0.0655722, 0.00689496],
         [-0.00528128, 0.992775, -0.119874, 0.121215],
         [0.0654112, 0.119962, 0.990621, 0.0605288], [0, 0, 0, 1]],
        dtype=np.float)


if __name__ == '__main__':
    main()