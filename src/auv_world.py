#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-03-14

import Box2D as b2
import config as C
import math
import random


class World:
    def __init__(self,
                 world_size=50,
                 gravity=-1,
                 num_obstacles=3,
                 obstacle_sizes=3,
                 obstacle_noise=1):

        # physics world
        self.world = b2.b2World(gravity=(0, gravity))

        # static objects in the scene that do not move
        # for drawing purposes
        # the drawer should scale these for visualization when drawing

        # creat a box around the working area
        # vertices are in physics coords.
        a = 1
        box_vertices = [(a, a), (world_size - a, a),
                        (world_size - a, world_size - a), (a, world_size - a)]

        # a static body. A body is just a point with no shape
        self._box = self.world.CreateStaticBody(
            shapes=b2.b2EdgeShape(vertices=box_vertices))
        # fixtures are added shapes to point-wise bodies.
        # these determine the collisions and such while the bodies contain
        # the speed and such
        for i in range(4):
            verts = [box_vertices[i], box_vertices[(i + 1) % 4]]
            self._box.CreateEdgeFixture(vertices=verts,
                                        density=0,
                                        friction=0)

        for i in range(num_obstacles):
            pos = [random.randint(0,world_size), random.randint(0,world_size)]
            verts = _make_random_obstacle(pos, obstacle_sizes, obstacle_noise)
            obs = self.world.CreateStaticBody(shapes=b2.b2PolygonShape(vertices=verts))

    def update(self, dt=1 / 60):
        self.world.Step(dt, 10, 10)


def _make_random_obstacle(pos, size=5, size_range=2, num_verts=10):
    angles = range(0,360,int(360/num_verts))
    verts = []
    base_x, base_y = pos
    for angle in angles:
        x = math.cos(angle)*(size+random.random()*size_range -size_range/2)
        y = math.sin(angle)*(size+random.random()*size_range -size_range/2)
        verts.append((base_x+x,base_y+y))
    return verts

