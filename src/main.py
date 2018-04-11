#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-03-14

from auv_world import World
from auv import AUV
from visualizer import Visualizer
from keyboard_controller import Controller

import config as C
import pygame as pg

if __name__ == '__main__':
    world = World(world_size=50, gravity=0)
    auv = AUV(world, x=25, y=5)

    viz = None
    viz = Visualizer(world, C.SCREEN_WIDTH, C.SCREEN_HEIGHT, C.PPM)

    cont = Controller(auv)

    running = True
    # keeps time nicely
    clock = pg.time.Clock()
    while running:
        running = cont.update(C.TIME_STEP)
        auv.update(C.TIME_STEP)
        # if this is never called, the visualizer wont show the lines
        auv.get_proximity()
        world.update(C.TIME_STEP)
        if viz is not None:
            viz.update(points=auv.last_casted_points,
                       points_connection=auv.get_position())
        clock.tick(C.TARGET_FPS)

    pg.quit()
