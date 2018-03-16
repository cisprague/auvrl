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
                 thruster_limit = 30):
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
                fixtures = b2.b2FixtureDef(
                    shape = b2.b2PolygonShape(vertices=auv_vertices),
                    density=10,
                    # slidey
                    friction=0.1,
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

        # PID controller for the joint motor speed
        self._thruster_PID = Pid.PID(1,0,0)

        # current target angle of the thrust
        # in degrees
        self._target_thrust_angle = 0


    def get_position(self):
        return self._auv.position

    def get_thruster_angle(self):
        """
        returns degrees
        """
        return self._thruster.angle*C.RADTODEG


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
        self._thruster.ApplyForceToCenter(force = self._thrust, wake=True)

        # TODO PID
        if self._thruster_joint.angle > self._target_thrust_angle + 0.05:
            self._thruster_joint.motorSpeed = -1

        elif self._thruster_joint.angle < self._target_thrust_angle - 0.05:
            self._thruster_joint.motorSpeed = 1

        else:
            self._thruster_joint.motorSpeed = 0




