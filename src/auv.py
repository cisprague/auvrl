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

    # physics world
    world = b2.b2World(gravity=(0,-1))

    # static objects in the scene that do not move
    # for drawing purposes
    # the drawer should scale these for visualization when drawing
    static_objects = []
    dynamic_objects = []

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
                            friction = 0.5)

    # add the floor to be drawn later
    static_objects.append(floor)

    # auv starting position
    auv_x = 10
    auv_y = floor_y + 5
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

    dynamic_objects.append(auv)


    # where the thruster is attached to the auv
    thruster_local_anchor_auv = (0,0)

    # a simple triangle
    auv_thruster_vertices = [
        (0,0),
        (-1,-1),
        (-1,1)
    ]

    auv_thruster = world.CreateDynamicBody(
                    position=(auv_x-auv_l/2, auv_y),
                    angle=0,
                    fixtures=b2.b2FixtureDef(
                        shape=b2.b2PolygonShape(vertices=auv_thruster_vertices),
                        #  shape=b2.b2EdgeShape(vertices=auv_thruster_vertices),
                        density=1,
                        friction=0.1,
                        restitution=0))

    dynamic_objects.append(auv_thruster)

    joints =[]

    thruster_joint = b2.b2RevoluteJointDef(
                        bodyA=auv,
                        bodyB=auv_thruster,
                        localAnchorA=auv_local_anchor_thruster,
                        localAnchorB=thruster_local_anchor_auv,
                        enableMotor=False,
                        enableLimit=True,
                        maxMotorTorque=1
                        )
    # dont let the thruster collide with the auv body
    thruster_joint.collideConnected = False
    # rotate between these angles hopefully
    thruster_joint.lowerAngle = -np.pi/6
    thruster_joint.upperAngle = np.pi/6
    auv_thruster.joint = world.CreateJoint(thruster_joint)

    joints.append(thruster_joint)


    # setup a simple pygame screen
    PPM = 20.0  # pixels per meter
    TARGET_FPS = 60
    TIME_STEP = 1.0 / TARGET_FPS
    SCREEN_WIDTH, SCREEN_HEIGHT = 640, 480

    screen = pg.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT),0,32)
    pg.display.set_caption('AUV test')
    clock = pg.time.Clock()

    running = True
    while running:
        # Check the event queue
        for event in pg.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                # The user closed the window or pressed escape
                running = False

            if event.type == KEYDOWN and event.key == 273: # UP key
                auv.ApplyForceToCenter(force=(0,10),wake=True)


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
