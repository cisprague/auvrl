# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

import numpy as np

class Dynamics(object):

    '''
    Indirect optimal control implementation (via Pontryagin's minimum principle)
    for a thrust vectored rod subject to spherical fluid resistance
    '''

    def __init__(self, g=9.807, T=10, m=10, l=3, rho=1000, cd=0.3, A=1):

        # parameters
        self.g   = g   # gravity [m/s]
        self.T   = T   # max thrust [N]
        self.m   = m   # mass [kg]
        self.l   = l   # length [m]
        self.rho = rho # fluid density [kg/m^3]
        self.cd  = cd  # drag coefficient [ND]
        self.A   = A   # planaform area

        # control bounds
        self.ulb  = [0, -10]
        self.uub  = [1, 10]

        # state and action space dimensions
        self.udim = 2
        self.sdim = 6

    def eom_fullstate(self, fullstate, control):

        # extract fullstate and control
        x, y, vx, vy, theta, omega, lx, ly, lvx, lvy, ltheta, lomega = fullstate
        u, phi = control

        # subexpression elimination
        x0 = self.T*u
        x1 = phi + theta
        x2 = np.cos(x1)
        x3 = vx**2
        x4 = vy**2
        x5 = np.sqrt(x3 + x4)
        x6 = self.A*self.cd*self.rho*x5/2
        x7 = np.sin(x1)
        x8 = 1/(2*x5)
        x9 = self.A*self.cd*self.rho*vx*vy
        x10 = 2*x5
        x11 = self.A*self.cd*self.rho

        # state transition
        return np.array([
            vx,
            vy,
            vx*x6 + x0*x2,
            -self.g + vy*x6 + x0*x7,
            omega,
            -6*x0*np.sin(phi)/(self.l*self.m),
            0,
            0,
            -x8*(lvx*x11*(2*x3 + x4) + lvy*x9 + lx*x10),
            -x8*(lvx*x9 + lvy*x11*(x3 + 2*x4) + ly*x10),
            x0*(lvx*x7 - lvy*x2),
            -ltheta
        ])

    def hamiltonian(self, fullstate, control, alpha=0):

        # extract fullstate and control
        x, y, vx, vy, theta, omega, lx, ly, lvx, lvy, ltheta, lomega = fullstate
        u, phi = control

        # subexpression elimination
        x0 = self.T*u
        x1 = phi + theta
        x2 = np.cos(x1)
        x3 = self.A*self.cd*self.rho*np.sqrt(vx**2 + vy**2)/2
        x4 = np.sin(x1)

        # hamiltonian
        return alpha*u + ltheta*omega + lvx*(vx*x3 + x0*x2) + \
        lvy*(-self.g + vy*x3 + x0*x4) + lx*vx + ly*vy + u**2*(-alpha + 1) + \
        6*lomega*x0*(x2*np.sin(theta) - x4*np.cos(theta))/(self.l*self.m)

    def pontryagin(self, fullstate, alpha=0, bound=True):

        # extract fullstate
        x, y, vx, vy, theta, omega, lx, ly, lvx, lvy, ltheta, lomega = fullstate

        # subexpression elimination
        x0 = self.l*self.T*self.m
        x2 = self.l**2
        x3 = 12*lomega
        x4 = lvx*x2 + x3*np.sin(theta)
        x5 = lvy*x2
        x6 = x3*np.cos(theta)
        x7 = 1/np.sqrt(x4**2 + (x5 - x6)**2)

        # optimal thrust unit vector wrt global
        txstar = -x4*x7
        tystar = x7*(-x5 + x6)

        # optimal thrust angle wrt body
        phistar = np.arctan2(tystar, txstar) - theta

        # optimal thrust magnitude
        x1 = phistar + theta
        ustar = (-6*self.T*lomega*np.sin(phistar) + alpha*(self.l*self.m) + lvx*x0*np.cos(x1) + lvy*x0*np.sin(x1))/(2*self.l*self.m*(alpha - 1))

        # optimal controls
        return np.array([ustar, phistar])
