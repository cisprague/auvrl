# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

from scipy.integrate import ode
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


class Leg(object):

    def __init__(self, dynamics, alpha=0, bound=True, freetime=True):

        # dynamical system
        self.dynamics = dynamics

        # homotopy parametre
        self.alpha = alpha

        # control bound
        self.bound = bound

        # infinite time horizon
        self.freetime = freetime

        # equality constraints
        self.nec = self.dynamics.sdim
        if self.freetime: self.nec += 1

        # numerical integrator
        self.integrator = ode(
            lambda t, fs: self.dynamics.eom_fullstate(
                fs, self.dynamics.pontryagin(fs, self.alpha, self.bound)
            ),
            #lambda t, fs: self.dynamics.eom_fullstate_jac(
            #    fs, self.dynamics.pontryagin(fs, self.alpha, self.bound)
            #)
        )

    def recorder(self, t, fs):

        # times
        self.times = np.append(self.times, t)

        # fullstates
        self.states = np.vstack((self.states, fs))

        # actions
        self.actions = np.vstack((
            self.actions,
            self.dynamics.pontryagin(fs, self.alpha, bound=self.bound)
        ))

    def set_times(self, t0, tf):
        self.t0 = float(t0)
        self.tf = float(tf)

    def set_states(self, s0, sf):
        self.s0 = np.array(s0, dtype=float)
        self.sf = np.array(sf, dtype=float)

    def set_costates(self, l0):
        self.l0 = np.array(l0, dtype=float)

    def set(self, t0, s0, l0, tf, sf):
        self.set_times(t0, tf)
        self.set_states(s0, sf)
        self.set_costates(l0)

    def set_params(self, alpha, bound):

        # homotopy parametre
        self.alpha = alpha

        # control bounds
        self.bound = bool(bound)

        # equality consraints
        self.nec = self.dynamics.sdim
        if self.freetime: self.nec += 1

    def propagate(self, atol=1e-5, rtol=1e-5):

        # departure fullstate
        fs0 = np.hstack((self.s0, self.l0))

        # reset trajectory records
        self.times = np.empty((1, 0))
        self.states = np.empty((0, self.dynamics.sdim*2))
        self.actions = np.empty((0, self.dynamics.udim))

        # set integration method
        self.integrator.set_integrator(
            "dop853", atol=atol, rtol=rtol, verbosity=1
        )

        # set recorder
        self.integrator.set_solout(self.recorder)

        # set departure configuration
        self.integrator.set_initial_value(fs0, self.t0)

        # numerically integrate
        self.integrator.integrate(self.tf)

    def mismatch(self, atol=1e-5, rtol=1e-5):

        # propagte trajectory
        self.propagate(atol=atol, rtol=rtol)

        # final state mismatch equality constraint
        ceq = self.states[-1, 0:self.dynamics.sdim] - self.sf

        # infinite time horizon
        if self.freetime:

            # final Hamiltonian
            H = self.dynamics.hamiltonian(
                self.states[-1], self.actions[-1], self.alpha
            )

            # append equality constraint
            ceq = np.hstack((ceq, [H]))

        return ceq

    def plot_traj(self, ax=None, mark="k.-", quiver=True):

        # create new axes if not supplied
        if ax is None:
            fig = plt.figure()
            ax = fig.gca(projection="3d")

        # plot the positions
        ax.plot(*[self.states[:, dim] for dim in [0, 1, 2]], mark)

        # show thrust profile if desired
        if quiver:

            # quaternions
            qr, qx, qy, qz = [self.states[:, dim] for dim in [6, 7, 8, 9]]

            # Rene Descartes directions
            utx = 2 * (np.multiply(qx, qz) - np.multiply(qy, qr))
            uty = 2 * (np.multiply(qy, qz) - np.multiply(qx, qr))
            utz = 1 - 2 * (np.square(qx) + np.square(qy))

            # thrust magnitudes
            utx, uty, utz = [np.multiply(self.actions[:, 0], i) for i in [utx, uty, utz]]

            # plot thrusts
            ax.quiver(
                *[self.states[:, dim] for dim in [0, 1, 2]],
                utx, uty, utz,
                normalize=True,
                length=0.5
            )

        return ax

    def plot_states(self):

        # create subplots
        f, ax = plt.subplots(self.dynamics.sdim, 2, sharex=True)

        # get states and costates
        s = self.states[:, :self.dynamics.sdim]
        c = self.states[:, self.dynamics.sdim:self.dynamics.sdim*2]

        # plot data
        for i in range(self.dynamics.sdim):
            ax[i, 0].plot(self.times, s[:, i], "k.-")
            ax[i, 1].plot(self.times, c[:, i], "k.-")

        return ax

    def plot_actions(self):

        # create subplots
        f, ax = plt.subplots(self.dynamics.udim, sharex=True)

        for i in range(self.dynamics.udim):
            ax[i].plot(self.times, self.actions[:, i], "k.-")

        return ax

if __name__ == "__main__":

    from dynamics import Dynamics

    # instantiate AUV
    sys = Dynamics()

    # instantiate leg
    leg = Leg(sys)

    # arbitrary boundaries
    t0 = 0
    s0 = np.array([50, 50, 1, 1, 1, 1])
    l0 = np.random.randn(len(s0))
    tf = 10000
    sf = np.random.randn(len(s0))

    # set TPBVP problem boundires
    leg.set(t0, s0, l0, tf, sf)


    print(leg.mismatch(atol=1e-10, rtol=1e-10))
    
    leg.plot_states()
    leg.plot_actions()
    plt.show()
