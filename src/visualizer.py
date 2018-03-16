#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-03-14

import pygame as pg

class Visualizer:
    def __init__(self,
                 world,
                 w=640,
                 h=480,
                 ppm=20):

        self._ppm = ppm
        self._w = w
        self._h = h

        self._screen = pg.display.set_mode((w,h),0,32)
        self._world = world.world

    def update(self):
        # refresh
        self._screen.fill((0,0,0,0))

        for body in self._world.bodies:
            for fixture in body.fixtures:
                shape = fixture.shape
                # transform vertices according to body
                vertices = [(body.transform * v) * self._ppm for v in shape.vertices]
                # pygame uses screen coords while box2D uses math coords, flip y
                vertices = [(v[0], self._h - v[1]) for v in vertices]
                # draw stuff
                pg.draw.lines(self._screen, (255,255,255,0), True, vertices)

            # draw a small circle for the center of the object
            c = body.position*self._ppm
            c[1] = self._h - c[1]
            pg.draw.circle(self._screen, (255,0,0,0), (int(c[0]),int(c[1])), 2)

        # update screen
        pg.display.flip()

    def kill(self):
        pg.kill()
        print('Done')

