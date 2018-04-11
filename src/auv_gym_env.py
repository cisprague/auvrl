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
import random


class Environment:
    def __init__(self,
                 world_size, gravity, num_obstacles, obstacle_sizes, obstacle_noise,
                 xinit, yinit,
                 targetx, targety,
                 randomx=0, randomy=0):
        """
        world size in meters, origin at bottom left
        gravity in N/m tuple (x, y)
        num_obstacles is the number of random obstacles to be added to the world
        obstacle_sizes is the radius of the obstacles
        obstacle_noise makes the obstacles non-circular by adding/subtracting a random value < obstacle_noise

        xinit,yinit, starting position of the auv
        targetx, targety, target position for the auv

        randomx, randomy, uniform random addition to xinit,yinit between (-random,random)
        """
        # for resetting later
        self.args = [world_size, gravity, num_obstacles, obstacle_sizes, obstacle_noise,\
                     xinit, yinit,\
                     targetx, targety,\
                     randomx, randomy]
        self.reset()

    def reset(self):
        world_size, gravity, num_obstacles, obstacle_sizes, obstacle_noise, xinit, yinit, targetx, targety, randomx, randomy = self.args

        xinit += (random.random() - 0.5) * randomx
        yinit += (random.random() - 0.5) * randomy

        self.world = World(world_size, gravity, num_obstacles, obstacle_sizes, obstacle_noise)
        self.auv = AUV(self.world, xinit, yinit)
        self.viz = None
        self.viz = Visualizer(self.world, C.SCREEN_WIDTH,
                              C.SCREEN_HEIGHT, C.PPM)
        self.cont = Controller(self.auv)
        self.clock = pg.time.Clock()

        self.target_point = [targetx, targety]

        return self._observe()

    def _observe(self):
        pos = self.auv.get_position()
        dist = G.euclid_distance(pos, self.target_point)
        heading = self.auv.get_heading()
        angle = G.directed_angle([1, 0], pos)
        prox = self.auv.get_proximity()

        return dist, heading, angle, prox

    def _reward(self, obs, action):

        # initialise reward
        r = 0
        # world size
        D = self.args[0]

        # distance to target
        d = obs[0]
        # normalised distance to target
        d /= D
        # reward being closer to target
        r += 1 / math.exp(d)

        # ray observations
        rays = obs[3]
        # number of rays
        nrays = len(rays)
        # initialise ray reward
        rr = 0
        # for each sensor array
        for ray in rays:
            # ray distance observation
            d = ray[0]
            # if ray doesn't sense anything
            if d == -1:
                # positive reward, because no obstacles are sensed
                rr += 1
            # if ray does sense something
            else:
                # normalised distance to obstacle
                d /= D
                # negatively reward getting closer to obstacle
                rr -= 1 / math.exp(d)
        # average ray rewards
        rr /= nrays
        # add ray awards to overall rewards
        r += rr

        # extract actions
        thrust_angle, thrust_power = action
        # normalise thruster power
        thrust_power /= 50
        # negatively reward using thruster (optimal control)
        r -= thrust_power**2

        # return overall reward
        return r

    def _done(self):
        done = False
        pos = self.auv.get_position()
        if math.fabs(pos[0] - self.target_point[0]) < C.TARGET_AREA and\
           math.fabs(pos[1] - self.target_point[1]) < C.TARGET_HEIGHT:
            done = True

        colls = self.auv.get_collisions()
        if colls is not None and len(colls) > 1:
            done = True

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
        reward = self._reward(obs, action)
        done = self._done()
        info = {}

        return obs, reward, done, info

    def render(self):
        self.auv.get_proximity()
        self.viz.update(points=self.auv.last_casted_points,
                        points_connection=self.auv.get_position())

def make_environment(env_type, world_size, gravity, randomx, randomy):
    """
    env_type is one of ['empty','fewlarge','manysmall']
    """

    if env_type=='empty':
        num_obstacles = 0
        obstacle_sizes = 1
        obstacle_noise = 1
    if env_type=='fewlarge':
        num_obstacles = 2
        obstacle_sizes = 12
        obstacle_noise = 5
    if env_type=='manysmall':
        num_obstacles = 13
        obstacle_sizes = 2
        obstacle_noise = 2

    env = Environment(world_size,gravity,num_obstacles,obstacle_sizes,obstacle_noise,
                      10,world_size-10,
                      world_size/2,0,
                      randomx,randomy)
    return env

if __name__=='__main__':
    env = make_environment('fewlarge',50,-1,5,5)
    for i in range(10000):
        env.step((0,0))
        env.render()
