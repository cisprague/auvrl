# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

import numpy as np
import _dynamics

class Dynamics(object):

    '''
    Indirect optimal control transcription (via Pontryagin's minimum principle)
    for a thrust vectored rod subject to spherical fluid resistance
    '''

    def __init__(self, g=0, T=20, m=10, l=3):

        # parameters
        self.params = [g, T, m, l]

        # state and action/control dimensions
        self.sdim = 6
        self.adim = 3

        # control bounds
        self.ulb = [0, np.deg2rad(-10)]
        self.uub = [1, np.deg2rad(10)]

    # computes the instantaneous cost
    def lagrangian(self, control, alpha=0):
        return _dynamics.lagrangian(*control, alpha)

    # used for infinite time-horizon boundary condition (H_f = 0)
    def hamiltonian(self, fullstate, control, alpha=0):
        return _dynamics.hamiltonian(*fullstate, *control, alpha, *self.params)

    # computes the instantaneous optimal control
    def pontryagin(self, fullstate, alpha=0, bound=True):

        control = _dynamics.pontryagin(*fullstate, alpha, *self.params).flatten()

        if bound:
            # unpack controls
            ut, ux, uy = control
            # apply thrust throttle bound
            ut = min(max(self.ulb[0], ut), self.uub[0])
            # thrust angle wrt global
            gamma = np.arctan2(uy, ux)
            # auv orientation
            theta = fullstate[4]
            # thrust angle wrt body
            phi = gamma - theta
            # apply thrust vectoring bounds
            phi = min(max(self.ulb[1], phi), self.uub[1])
            # thrust angle wrt global
            gamma = theta + phi
            # thrust unit vector wrt global
            ux, uy = np.cos(gamma), np.sin(gamma)
            # return bounded controls
            return np.array([ut, ux, uy])
        else:
            # return unbounded control
            return control


    # state and costate dynamics
    def eom_fullstate(self, fullstate, control):
        return _dynamics.eom_fullstate(*fullstate, *control, *self.params).flatten()

    # jacobian of dynamics for integration scheme
    def eom_fullstate_jac(self, fullstate, control):
        return _dynamics.eom_fullstate_jac(*fullstate, control, *self.params)
