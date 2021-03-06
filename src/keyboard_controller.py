#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-03-14

import pygame as pg
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)

import config as C


class Controller:
    def __init__(self, auv):
        self._auv = auv

        self._angle_step = 10
        self._thrust = 20

        self._thrusting = False
        self._target_thrust_angle = 0

    def update(self, dt):
        """
        return False if the sim should continue otherwise return True.

        # state of the auv:
        #  x,y = self._auv.get_position()
        #  vx,vy = self._auv.get_velocity()
        #  degrees = self._auv.get_heading()
        #  degrees_thr = self._auv.get_thruster_angle()
        #  list_of_dists_points = self._auv.get_proximity()
        #  list_of_things = self._auv.get_collisions()
        # control of the auv:
        #  self._auv.set_thrust(amount)
        #  self._auv.set_thrust_angle(degrees)


        """

        for event in pg.event.get():
            et = event.type
            if et == QUIT or (et == KEYDOWN and event.key == K_ESCAPE):
                return True

            # toggle thrusting
            if et == KEYDOWN and event.key == 273:  # UP KEY
                self._thrusting = not self._thrusting

            if et == KEYDOWN and event.key == 275:  # RIGHT KEY
                self._target_thrust_angle += self._angle_step
                self._auv.set_thrust_angle(self._target_thrust_angle)

            if et == KEYDOWN and event.key == 276:  # LEFT KEY
                self._target_thrust_angle -= self._angle_step
                self._auv.set_thrust_angle(self._target_thrust_angle)

        if self._target_thrust_angle > 0:
            self._target_thrust_angle = min(
                self._auv._thruster_limit, self._target_thrust_angle)
        if self._target_thrust_angle < 0:
            self._target_thrust_angle = max(-self._auv._thruster_limit,
                                            self._target_thrust_angle)

        if self._thrusting:
            self._auv.set_thrust(self._thrust)
        else:
            self._auv.set_thrust(0)

        return False
