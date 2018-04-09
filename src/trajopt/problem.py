# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

from leg import Leg
import numpy as np
import pygmo as pg

class Problem(object):

    def __init__(self, leg, atol=1e-10, rtol=1e-10):

        # trajectory leg
        self.leg = leg

        # numerical integration tolerances
        self.atol = atol
        self.rtol = rtol

    def get_nobj(self):
        return 1

    def get_nec(self):
        return self.leg.nec

    def get_bounds(self):
        lb = [1] + [-100]*self.leg.dynamics.sdim
        ub = [10000] + [100]*self.leg.dynamics.sdim
        return (lb, ub)

    def fitness(self, z):

        # set leg times
        self.leg.set_times(0, z[0])

        # set costates
        self.leg.set_costates(z[1:])

        # equality constraints
        ceq = self.leg.mismatch(atol=self.atol, rtol=self.rtol)

        # return fitness vector
        return np.hstack(([1], ceq))

    def gradient(self, z):
        return pg.estimate_gradient(self.fitness, z)
