# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

class Dynamics(object):

    def __init__(self, g=9.807, T=10, m=10, l=3, cd=0.3, A=1):

        # parameters
        self.g = g
        self.T = T
        self.m = m
        self.l = l
        self.cd = cd
        self.A = A

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

        # fullstate transition
        return np.array([
            vx,
            vy,
            vx*x6 + x0*x2,
            -self.g + vy*x6 + x0*x7,
            omega,
            -12*x0*np.sin(phi)/(self.l**2*self.m),
            0,
            0,
            -x8*(lvx*x11*(2*x3 + x4) + lvy*x9 + lx*x10),
            -x8*(lvx*x9 + lvy*x11*(x3 + 2*x4) + ly*x10),
            x0*(lvx*x7 - lvy*x2),
            -ltheta
        ])

    def pontryagin(self, fullstate, alpha=0):

        # extract fullstate
        x, y, vx, vy, theta, omega, lx, ly, lvx, lvy, ltheta, lomega = fullstate

        # subexpression elimination
        x0 = self.l**2
        x1 = 12*lomega
        x2 = self.T*self.m
        x3 = phi + theta
        x4 = lvx*x0 + x1*np.sin(theta)
        x5 = lvy*x0
        x6 = x1*np.cos(theta)
        x7 = 1/np.sqrt(x4**2 + (x5 - x6)**2)

        # optimal thrust magnitude
        ustar = -self.T*x1*np.sin(phi) + alpha*self.m*x0 + lvx*x0*x2*np.cos(x3) + lvy*x0*x2*np.sin(x3))/(2*self.m*x0*(alpha - 1))

        # optimal thrust direction
        txstar = -x4*x7
        tystar = x7*(-x5 + x6)
        phistar = np.arctan()

        # optimal controls
        return np.array([ustar, txstar, tystar])
