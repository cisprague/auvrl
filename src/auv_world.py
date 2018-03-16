#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-03-14

import numpy as np
import Box2D as b2

import config as C


class World:
    def __init__(self,
                 world_size = 50,
                 gravity = -1):

        # physics world
        self.world = b2.b2World(gravity=(0,gravity))

        # static objects in the scene that do not move
        # for drawing purposes
        # the drawer should scale these for visualization when drawing

        # creat a box around the working area
        # vertices are in physics coords.
        a = 1
        box_vertices = [ (a,a), (world_size-a,a), (world_size-a, world_size-a), (a, world_size-a)]

        # a static body. A body is just a point with no shape
        self._box = self.world.CreateStaticBody(shapes=b2.b2EdgeShape(vertices=box_vertices))
        # fixtures are added shapes to point-wise bodies.
        # these determine the collisions and such while the bodies contain
        # the speed and such
        for i in range(4):
            verts = [box_vertices[i], box_vertices[(i+1)%4]]
            self._box.CreateEdgeFixture(vertices = verts,
                                        density = 0,
                                        friction = 0)

        obs = self.world.CreateStaticBody(shapes=b2.b2PolygonShape(vertices=[(20,20), (25,30), (18,22), (12,10)]))
        obs = self.world.CreateStaticBody(shapes=b2.b2PolygonShape(vertices=[(10,40), (35,44), (22,42), (25,40)]))

    def update(self, dt=1/60):
        self.world.Step(dt, 10, 10)
