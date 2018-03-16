#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-03-14

import math

# deg = rad * RADTODEG
RADTODEG = 360/(math.pi*2)

# update frequency of the screen
TARGET_FPS = 60
TIME_STEP = 1.0 / TARGET_FPS

PPM = 20.0  # pixels per meter
SCREEN_WIDTH, SCREEN_HEIGHT = 1000, 1000
