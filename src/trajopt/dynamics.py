# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

import numpy as np
import _dynamics

class Dynamics(object):

    '''
    Indirect optimal control transcription (via Pontryagin's minimum principle)
    for a thrust vectored rod subject to spherical fluid resistance
    '''

    def __init__(self, g=9.807, T=10, m=10, l=3, rho=1000, cd=0.3, A=1):

        # parameters
        self.params = [g, T, m, l, rho, cd, A]

        # state and action/control dimensions
        self.sdim = 6
        self.adim = 3

        # control bounds
        self.ulb = [0, -10]
        self.uub = [1, 10]

        # define nondimensional units
        self.L  = 1000
        self.V  = 100
        self.A  = self.V**2/self.L
        self.M  = m
        self.F  = self.M*self.A
        self.D  = rho
        self.SL = self.L**2
        self.AN = np.pi
        self.ON = self.V/self.L

    def nondimensionalise(self, state):

        # unpack parameters
        g, T, m, l, rho, cd, A = self.params

        # nondimensionalise params
        g   /= self.A
        T   /= self.F
        m   /= self.M
        l   /= self.L
        rho /= self.D
        A   /= self.SL

        # pack params
        self.params = [g, T, m, l, rho, cd, A]

        # unpack state
        x, y, vx, vy, theta, omega = state

        # nondimensionalise state
        x     /= self.L
        y     /= self.L
        vx    /= self.V
        vy    /= self.V
        theta /= self.AN
        omega /= self.ON

        # pack state
        return np.array([x, y, vx, vy, theta, omega])

    def dimensionalise(self, state):

        # unpack parameters
        g, T, m, l, rho, cd, A = self.params

        # nondimensionalise params
        g   *= self.A
        T   *= self.F
        m   *= self.M
        l   *= self.L
        rho *= self.D
        A   *= self.SL

        # pack params
        self.params = [g, T, m, l, rho, cd, A]

        # unpack state
        x, y, vx, vy, theta, omega = state

        # nondimensionalise state
        x     *= self.L
        y     *= self.L
        vx    *= self.V
        vy    *= self.V
        theta *= self.AN
        omega *= self.ON

        # pack state
        return np.array([x, y, vx, vy, theta, omega])


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
