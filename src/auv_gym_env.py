#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-04-09



from auv_world import World
from auv import AUV
from visualizer import Visualizer
from keyboard_controller import Controller

from utils import geometry as G

import config as C
import pygame as pg

import math

class Environment:
    def __init__(self, world_size, gravity, xinit, yinit, targetx, targety):
        """
        world size in meters, origin at bottom left
        gravity in N/m tuple (x, y)


        xinit,yinit, starting position of the auv
        targetx, targety, target position for the auv
        """

        self.world = World(world_size,gravity)
        self.auv = AUV(self.world, xinit,yinit)
        self.viz = None
        self.viz = Visualizer(self.world, C.SCREEN_WIDTH, C.SCREEN_HEIGHT, C.PPM)
        self.cont = Controller(self.auv)
        self.clock = pg.time.Clock()

        self.target_point = [targetx, targety]

    def _observe(self):
        pos = self.auv.get_position()
        dist = G.euclid_distance(pos, self.target_point)
        angle = G.directed_angle([1,0], pos)
        prox = self.auv.get_proximity()

        return dist, angle, prox


    def _reward(self):
        # TODO reward function
        return 0

    def _done(self):
        done = False
        pos = self.auv.get_position()
        if math.fabs(pos[0] - self.target_point[0]) < C.TARGET_AREA and\
           math.fabs(pos[1] - self.target_point[1]) < C.TARGET_HEIGHT:
            done=True

        colls = self.auv.get_collisions()
        if colls is not None and len(colls) > 1:
            done=True

        return done


    def step(self, action):
        """
        action is a tuple of 'thrust angle' and 'thrust power'
        """
        thrust_angle, thrust_power = action
        self.auv.set_thrust_angle(thrust_angle)
        self.auv.set_thrust(thrust_power)
        self.auv.update(C.TIME_STEP)
        self.world.update(C.TIME_STEP)
        self.clock.tick(C.TARGET_FPS)

        obs = self._observe()
        reward = self._reward()
        done = self._done()
        info = {}

        return obs, reward, done, info

    def render(self):
        self.auv.get_proximity()
        self.viz.update(point=self.auv.last_casted_points,
                        points_connection=self.auv.get_position())