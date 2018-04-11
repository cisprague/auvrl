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

        self._screen = pg.display.set_mode((w, h), 0, 32)
        self._world = world.world

    def update(self, points=None, points_connection=None):
        # refresh
        self._screen.fill((0, 0, 0, 0))

        for body in self._world.bodies:
            for fixture in body.fixtures:
                shape = fixture.shape
                # transform vertices according to body
                vertices = [(body.transform * v) *
                            self._ppm for v in shape.vertices]
                # pygame uses screen coords while box2D uses math coords, flip y
                vertices = [(v[0], self._h - v[1]) for v in vertices]
                # draw stuff
                pg.draw.lines(self._screen, (255, 255, 255, 0), True, vertices)

            # draw a small circle for the center of the object
            c = body.position * self._ppm
            c[1] = self._h - c[1]
            pg.draw.circle(self._screen, (255, 0, 0, 0),
                           (int(c[0]), int(c[1])), 2)

        if points is not None:
            if points_connection is not None:
                pc = [points_connection[0], points_connection[1]]
                pc[0] *= self._ppm
                pc[1] *= self._ppm
                pc[1] = self._h - pc[1]
            for p in points:
                if p[0] is None:
                    continue
                pt = [p[0], p[1]]
                pt[0] = p[0] * self._ppm
                pt[1] = p[1] * self._ppm
                pt[1] = self._h - pt[1]
                if points_connection is None:
                    pg.draw.circle(self._screen, (0, 255, 0, 0),
                                   (int(pt[0]), int(pt[1])), 2)
                else:
                    pg.draw.lines(self._screen, (0, 255, 0, 0), True, [pc, pt])

        # update screen
        pg.display.flip()

    def kill(self):
        pg.kill()
        print('Done')
