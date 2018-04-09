# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

from dynamics import Dynamics
from leg import Leg
from problem import Problem
import pygmo as pg
import pygmo_plugins_nonfree as pg7
import numpy as np

def main():

    # instantiate dynamics
    dyn = Dynamics()

    # instantiate leg
    leg = Leg(dyn, alpha=0, bound=True)

    # set boudaries
    t0 = 0
    s0 = np.array([1000, 1000, *np.random.randn(4)], dtype=float)
    l0 = np.random.randn(len(s0))
    tf = 1000
    sf = np.zeros(len(s0))
    leg.set(t0, s0, l0, tf, sf)

    # define problem
    udp = Problem(leg, atol=1e-8, rtol=1e-8)
    prob = pg.problem(udp)

    # instantiate algorithm
    uda = pg7.snopt7(True, "/usr/lib/libsnopt7_c.so")
    uda.set_integer_option("Major iterations limit", 4000)
    uda.set_integer_option("Iterations limit", 40000)
    uda.set_numeric_option("Major optimality tolerance", 1e-2)
    uda.set_numeric_option("Major feasibility tolerance", 1e-4)
    algo = pg.algorithm(uda)

    # instantiate population with one chromosome
    pop = pg.population(prob, 1)
    #pop = pg.population(prob, 0)
    #pop.push_back([1000, *l0])

    # optimise
    pop = algo.evolve(pop)


if __name__ == "__main__":
    main()
