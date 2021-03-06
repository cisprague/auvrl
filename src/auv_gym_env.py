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
                 randomx, randomy, env_name):
        """
        world size in meters, origin at bottom left
        gravity in N/m tuple (x, y)
        num_obstacles is the number of random obstacles to be added to the world
        obstacle_sizes is the radius of the obstacles
        obstacle_noise makes the obstacles non-circular by adding/subtracting a random value < obstacle_noise

        xinit, yinit, starting position of the auv
        targetx, targety, target position for the auv

        randomx, randomy, uniform random addition to xinit, yinit between (-random,random)
        """
        # for resetting later
        self.args = [world_size, gravity, num_obstacles, obstacle_sizes, obstacle_noise,\
                     xinit, yinit,\
                     targetx, targety,\
                     randomx, randomy]
        self.state_dim = 12
        self.action_dim = 2
        self.action_bound = [1, 1]
        self.name = env_name
        self.reset()

    def reset(self):
        world_size, gravity, num_obstacles, obstacle_sizes, obstacle_noise, xinit, yinit, targetx, targety, randomx, randomy = self.args

        xinit += (random.random() - 0.5) * randomx
        yinit += (random.random() - 0.5) * randomy

        self.world = World(world_size, gravity, num_obstacles, obstacle_sizes, obstacle_noise, xinit=xinit, yinit=yinit, rinit=3, targetx=targetx, targety=targety)
        self.auv = AUV(self.world, xinit, yinit)
        if hasattr(self, "viz"):
            num = self.viz.num
        else:
            num = 0
        self.viz = None
        self.viz = Visualizer(self.world, C.SCREEN_WIDTH,
                              C.SCREEN_HEIGHT, C.PPM, name=self.name)

        self.viz.num = num
        self.cont = Controller(self.auv)
        self.clock = pg.time.Clock()
        self.time = 0

        self.target_point = [targetx, targety]
        self.collided = False
        self.landed = False

        return self._observe()

    def _observe(self):
        pos     = self.auv.get_position()
        dist    = G.euclid_distance(pos, self.target_point)
        heading = self.auv.get_heading()
        angle   = G.directed_angle([1, 0], pos)
        prox    = self.auv.get_proximity()
        angvel  = self.auv.get_angular_velocity()

        return dist, heading, angle, angvel, prox

    def _reward(self, obs, action):

        # initialise reward
        r = 0
        # time penalty
        #r -= 1

        # world size
        D = self.args[0]
        # distance to target
        d = obs[0]
        # normalised distance to target
        d /= D
        # reward being closer to target
        r += 1 - abs(d)

        '''
        # ray observations
        rays = obs[-1]
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
                pass
            # if ray does sense something
            else:
                # negatively reward getting closer to obstacle
                rr += max(min(abs(d), 1.0), 0.0) - 1
        # average ray rewards
        rr /= nrays
        # add ray awards to overall rewards
        r += rr
        '''


        # angular velocity
        w = obs[3]
        r += - abs(w)*0.1
        # extract actions
        thrust_angle, thrust_power = action
        # negatively reward using thruster (optimal control)
        r += - abs(thrust_power)*0.1



        # return overall reward
        return r

    def _done(self, r):
        done = False
        pos = self.auv.get_position()
        vel = self.auv.get_velocity()
        angvel = self.auv.get_angular_velocity()
        if math.fabs(pos[0] - self.target_point[0]) < C.TARGET_AREA and \
           math.fabs(pos[1] - self.target_point[1]) < C.TARGET_HEIGHT:
            done = True
            r += 1000
            return done, r

        colls = self.auv.get_collisions()
        if len(colls) != 0:
            done = True
            r -= 100
            return done, r

        return done, r

    def step(self, action, dt=0.1):
        """
        action is a tuple of 'thrust angle' and 'thrust power'
        thrust angle and power commands are both between -1 and 1.
        """

        # extract thrust angle and power [-1, 1]
        thrust_angle, thrust_power = action
        # transform thrust power into [0, 1]
        thrust_power = (thrust_power + 1)/2
        # transform nondimensional commands into metric ones
        thrust_angle *= self.auv._thruster_limit
        thrust_power *= self.auv._thruster_power_limit

        # set AUV thrust angle
        self.auv.set_thrust_angle(thrust_angle)
        # set AUV thrust level
        self.auv.set_thrust(thrust_power)

        if dt is not None:
            self.auv.update(dt)
            self.world.update(dt)
            self.time += dt
        else:
            self.auv.update(C.TIME_STEP)
            self.world.update(C.TIME_STEP)
            self.time += C.TIME_STEP
        #self.clock.tick(C.TARGET_FPS)

        obs = self._observe()
        reward = self._reward(obs, action)
        done, reward = self._done(reward)
        info = {}

        return obs, reward, done, info

    def render(self):
        self.clock.tick(C.TARGET_FPS)
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
        num_obstacles = 20
        obstacle_sizes = 2
        obstacle_noise = 2

    env = Environment(world_size,gravity,num_obstacles,obstacle_sizes,obstacle_noise,
    10,world_size-10,
    world_size/2,0,
    randomx,randomy, env_type)
    return env

if __name__=='__main__':
    env = make_environment('fewlarge',50,-1,5,5)
    for i in range(10000):
        env.step((0,0))
        env.render()
