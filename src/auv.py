#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Author: Ozer Ozkahraman (ozkahramanozer@gmail.com)
# Date: 2018-03-06

import numpy as np
import Box2D as b2
import pygame as pg
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)

if __name__=='__main__':

    RADTODEG = 360/(np.pi*2)

    # physics world
    world = b2.b2World(gravity=(0,-0.1))

    # static objects in the scene that do not move
    # for drawing purposes
    # the drawer should scale these for visualization when drawing

    # create a simple line for the seafloor
    floor_x = 0
    floor_y = 1
    floor_length = 100
    # vertices are in physics coords.
    floor_vertices = [(floor_x-floor_length,floor_y),(floor_x+floor_length,floor_y)]

    # a static body. A body is just a point with no shape
    floor = world.CreateStaticBody(
                    shapes=b2.b2EdgeShape(
                        vertices=floor_vertices))
    # fixtures are added shapes to point-wise bodies.
    # these determine the collisions and such while the bodies contain
    # the speed and such
    floor.CreateEdgeFixture(vertices = floor_vertices,
                            density = 0,
                            friction = 0.1)


    # auv starting position
    auv_x = 5
    auv_y = floor_y + 10
    auv_l = 3
    auv_h = 1
    # CCW, centered on the rectangle center
    auv_vertices = [
                    # top right
                    (auv_l/2, auv_h/2),
                    # top left
                    (-auv_l/2, auv_h/2),
                    # bottom left
                    (-auv_l/2, -auv_h/2),
                    # bottom right
                    (auv_l/2, -auv_h/2)
    ]
    # where the thruster will connect to the auv
    # takes the center of the auv as origin
    # offset a little
    auv_local_anchor_thruster = (-auv_l/2-0.2, 0)

    # dynamic point
    auv = world.CreateDynamicBody(
            position = (auv_x,auv_y),
            angle=0.,
            fixtures = b2.b2FixtureDef(
                shape = b2.b2PolygonShape(vertices=auv_vertices),
                density=10,
                # slidey
                friction=0.1,
                # not bouncy
                restitution=0.7))


    # where the thruster is attached to the auv
    thruster_local_anchor_auv = (0,0)

    # a simple triangle
    thruster_size = 0.5
    auv_thruster_vertices = [
        (0,0),
        (-thruster_size,-thruster_size),
        (-thruster_size,thruster_size)
    ]

    auv_thruster = world.CreateDynamicBody(
                    position=(auv_x-auv_l/2, auv_y),
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
    joint_angle_limit = 30 /RADTODEG
    thruster_joint.lowerAngle = -joint_angle_limit
    thruster_joint.upperAngle = joint_angle_limit


    # create the joint
    auv_thruster.joint = world.CreateJoint(thruster_joint)



    # setup a simple pygame screen
    PPM = 20.0  # pixels per meter
    TARGET_FPS = 60
    TIME_STEP = 1.0 / TARGET_FPS
    SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

    screen = pg.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT),0,32)
    pg.display.set_caption('AUV test')
    clock = pg.time.Clock()

    running = True

    thrusting = False
    target_thrust_angle = 0
    while running:
        # Check the event queue
        for event in pg.event.get():
            print('event:',event)
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                # The user closed the window or pressed escape
                running = False

            if event.type == KEYDOWN and event.key == 273: # UP key
                thrusting = not thrusting

            if event.type == KEYDOWN and event.key == 275: # right key
                target_thrust_angle += 10*(1/RADTODEG)

            if event.type == KEYDOWN and event.key == 276: # left key
                target_thrust_angle -= 10*(1/RADTODEG)

            if target_thrust_angle > 0:
                target_thrust_angle = min(target_thrust_angle,joint_angle_limit)
            else:
                target_thrust_angle = max(target_thrust_angle,-joint_angle_limit)

            print('thrust angle:',auv_thruster.angle*RADTODEG)
            print('target_thrust_angle:',target_thrust_angle*RADTODEG)
            print('joint motor speed:',thruster_joint.motorSpeed)
            print('thrusting:',thrusting)

        if auv_thruster.joint.angle > target_thrust_angle + 0.05:
            auv_thruster.joint.motorSpeed = -1
        if auv_thruster.joint.angle < target_thrust_angle - 0.05:
            auv_thruster.joint.motorSpeed = 1


        if thrusting:
            thrust = 20
            tx = thrust * np.cos(auv_thruster.angle)
            ty = thrust * np.sin(auv_thruster.angle)
            auv_thruster.ApplyForceToCenter(force = (tx,ty), wake=True)



        # refresh
        screen.fill((0,0,0,0))

        for body in world.bodies:
            for fixture in body.fixtures:
                shape = fixture.shape
                # transform vertices according to body
                vertices = [(body.transform * v) * PPM for v in shape.vertices]
                # pygame uses screen coords while box2D uses math coords, flip y
                vertices = [(v[0], SCREEN_HEIGHT - v[1]) for v in vertices]
                # draw stuff
                pg.draw.lines(screen, (255,255,255,0), True, vertices)

            # draw a small circle for the center of the object
            c = body.position*PPM
            c[1] = SCREEN_HEIGHT - c[1]
            pg.draw.circle(screen, (255,0,0,0), (int(c[0]),int(c[1])), 2)


        # advance sim time
        world.Step(TIME_STEP, 10, 10)

        # update screen
        pg.display.flip()
        clock.tick(TARGET_FPS)

pg.quit()
print('quitted')
