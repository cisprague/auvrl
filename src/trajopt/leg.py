# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

from scipy.integrate import ode
import numpy as np
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
            lambda t, fs: self.dynamics.eom_fullstate_jac(
                fs, self.dynamics.pontryagin(fs, self.alpha, self.bound)
            )
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
        self.actions = np.empty((0, self.dynamics.adim))

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

    def plot_traj(self, ax=None, mark="k.-"):

        # create new axes if not supplied
        if ax is None:
            f, ax = plt.subplots(1)

        # plot the positions
        ax.plot(*[self.states[:, dim] for dim in [0, 1]], mark)

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

    def plot_actions(self, time=True):

        # create subplots
        f, ax = plt.subplots(self.dynamics.adim, sharex=True)

        for i in range(self.dynamics.adim):
            if time:
                ax[i].plot(self.times, self.actions[:, i], "k.-")
            else:
                ax[i].plot(self.actions[:, i], "k.-")

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

    leg.plot_traj()
    leg.plot_states()
    leg.plot_actions()
    plt.show()
