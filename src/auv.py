#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-03-06

import config as C
import Box2D as b2
import numpy as np

from utils import Pid


class AUV:
    def __init__(self,
                 world,
                 x = 10,
                 y = 10,
                 l = 3,
                 h = 1,
                 thruster_size = 0.5,
                 thruster_limit = 30,
                 ray_length = 10):
        """
        x,y defines the starting position of the AUV. 0,0 is bottom left.
        l,h length and height of the AUV.
        thruster_size is the size of the triangular thruster attached to the auv
        thruster_limit limits the rotation of the thruster to +/- this many degrees.
        """

        # CCW, centered on the rectangle center
        auv_vertices = [
                        # top right
                        (l/2, h/2),
                        # top left
                        (-l/2, h/2),
                        # bottom left
                        (-l/2, -h/2),
                        # bottom right
                        (l/2, -h/2)
        ]
        # where the thruster will connect to the auv
        # takes the center of the auv as origin
        # offset a little
        auv_local_anchor_thruster = (-l/2-0.2, 0)

        # dynamic point
        auv = world.world.CreateDynamicBody(
                position = (x,y),
                angle=0.,
                linearDamping=0.2,
                angularDamping=0.2,
                fixtures = b2.b2FixtureDef(
                    shape = b2.b2PolygonShape(vertices=auv_vertices),
                    density=10,
                    # slidey
                    friction=0.9,
                    # not very bouncy
                    restitution=0.7))


        # where the thruster is attached to the auv
        thruster_local_anchor_auv = (0,0)

        # a simple triangle
        auv_thruster_vertices = [
            (0,0),
            (-thruster_size,-thruster_size),
            (-thruster_size,thruster_size)
        ]

        auv_thruster = world.world.CreateDynamicBody(
                        position=(x-l/2, y),
                        angle=0,
                        fixtures=b2.b2FixtureDef(
                            shape=b2.b2PolygonShape(vertices=auv_thruster_vertices),
                            density=1,
                            friction=0.1,
                            restitution=0))


        thruster_joint = b2.b2RevoluteJointDef(
                            bodyA=auv,
                            bodyB=auv_thruster,
                            localAnchorA=auv_local_anchor_thruster,
                            localAnchorB=thruster_local_anchor_auv,
                            enableMotor=True,
                            enableLimit=True,
                            maxMotorTorque=10)

        # dont let the thruster collide with the auv body
        thruster_joint.collideConnected = False
        # rotate between these angles hopefully
        joint_angle_limit = thruster_limit / C.RADTODEG
        thruster_joint.lowerAngle = -joint_angle_limit
        thruster_joint.upperAngle = joint_angle_limit

        # create the joint
        auv_thruster.joint = world.world.CreateJoint(thruster_joint)


        # for future reference
        self._thruster_joint = auv_thruster.joint
        self._thruster = auv_thruster
        self._auv = auv
        self._thruster_limit = thruster_limit
        self._world = world

        # create rays for the proximity sensor of the AUV
        # these rays need to be rotated and moved with the auv body
        self._prox_rays = []
        num_rays = 8
        angle_offset = 45/2
        for i in range(num_rays):
            angle = i * 360/num_rays
            angle += angle_offset
            angle = angle % 360
            p1 = b2.b2Vec2([0,0])
            p2 = b2.b2Vec2([ray_length*np.cos(angle/C.RADTODEG),ray_length*np.sin(angle/C.RADTODEG)])
            self._prox_rays.append( (p1, p2) )

        # helper object for raycasting. Keeps a reference to auv transform in itself
        # together with the rays and world
        self._raycaster = Raycaster(self._world.world,
                                    self._prox_rays,
                                    self._auv.transform,
                                    ray_length)
        # for outside things (like visualizer) to get some points from the raycaster
        self.last_casted_points = None

        # current target angle of the thrust
        # in degrees
        self._target_thrust_angle = 0


    def get_position(self):
        return self._auv.position

    def get_velocity(self):
        return self._auv.linearVelocity

    def get_heading(self):
        return self._auv.angle*C.RADTODEG

    def get_thruster_angle(self):
        """
        returns degrees
        """
        return self._thruster.angle*C.RADTODEG

    def get_proximity(self):
        casted = self._raycaster.cast()
        self.last_casted_points = list(zip(*casted))[1]
        return casted

    def get_collisions(self):
        return self._auv.contacts


    def set_thrust(self,Nm):
        a = self.get_thruster_angle()/C.RADTODEG
        tx = Nm * np.cos(a)
        ty = Nm * np.sin(a)
        self._thrust = (tx,ty)

    def set_thrust_angle(self, deg):
        """
        set the target angle for the thruster
        deg in degrees
        """
        # clamp deg to the limits
        deg_sign = np.sign(deg)
        deg_mag = min(self._thruster_limit, np.abs(deg))
        deg = deg_sign * deg_mag
        self._target_thrust_angle = deg

    def update(self, dt):
        # apply the thrusters force to the whole body
        self._thruster.ApplyForceToCenter(force = self._thrust, wake=True)

        # adjust the thruster speed to reach the target angle
        if self._thruster_joint.angle > self._target_thrust_angle + 0.05:
            self._thruster_joint.motorSpeed = -1
        elif self._thruster_joint.angle < self._target_thrust_angle - 0.05:
            self._thruster_joint.motorSpeed = 1
        else:
            self._thruster_joint.motorSpeed = 0



class Raycaster(b2.b2RayCastCallback):
    def __init__(self, world, rays, transform, ray_length):
        super().__init__()
        self._last_frac = None
        self._last_point = None

        self._world = world
        self._rays = rays
        self._transform = transform
        self._ray_length = ray_length

    def ReportFixture(self, fixture, point, normal, fraction):
        if self._last_frac is None or fraction < self._last_frac:
            self._last_point = point
            self._last_frac = fraction
        return fraction


    def cast(self, normalize = True):
        casted = []
        for p1,p2 in self._rays:
            tp1 = self._transform*p1
            tp2 = self._transform*p2
            self._world.RayCast(self, tp1, tp2)
            if self._last_frac is not None and self._last_point is not None:
                if normalize:
                    casted.append((self._last_frac, self._last_point))
                else:
                    casted.append((self._last_frac*self._ray_length, self._last_point))

            else:
                casted.append((-1, (None,None)))
            self._last_frac = None
            self._last_point = None
        return casted





