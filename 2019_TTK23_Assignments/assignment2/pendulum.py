import numpy as np
from scipy.integrate import odeint
from scipy.linalg import solve_continuous_are as care
import matplotlib.pyplot as plt
from matplotlib import animation

class pendulum():
    """
    Environment for a pendulum, states are continous and given as the angle, and angular velocity, the actions
    are discrete, and indexed [0, 1, 2], corresponding to a torque of -1, +1 and the LQR regulator linearized 
    about the origin, with torque limits of [-1, +1]
    """
    def __init__(self, mass = 1, length = 1, x0 = [0, 0], gravity = 9.81):
        # Internal state vector
        self.x = np.array(x0)
        self.u = 0
        
        # Simulation parameters
        self.step_size = 0.05 
        
        # Internal parameters
        self.m = mass
        self.l = length
        self.g = gravity

        # Bounds on states and actions
        self.u_max = 5
        self.dt_max = 10
        
        # Pendulum differential equations
        self.dx = lambda t, dt, u : np.array([dt, self.g/self.l*np.sin(t) + u/(self.l*self.l*self.m)])

        # Find LQR regulator
        A = np.array([[0, 1], [self.g/self.l, 0]])
        B = np.array([[0], [1/(self.l*self.l*self.m)]])
        P = care(A, B, [[1, 0], [0, 1]], 1)
        self.K = B.T@P
    
    def reset(self, x0 = None):
        x0 = (2*np.random.rand(2) - 1)*np.array([np.pi, 1]) if x0 is None else x0
        self.x = np.array(x0)
        return self.state(), self.reward(), self.terminal()

    def state(self):
        return np.array([self.x[0], np.clip(self.x[1], -self.dt_max, self.dt_max)])

    def reward(self, x = None, u = None):
        x = self.state() if x is None else x
        u = self.u if u is None else u
        return -x[0]**2 - x[1]**2 - u**2
    
    def actions(self, x = None):
        return [0, 1, 2]

    def terminal(self, x = None):
        x = self.x if x is None else x
        return np.abs(x[1]) > self.dt_max or np.sum(x**2) < 1e-3
    
    def transition(self, state, action):
        # Scipy ODE solver for solving ODE
        f = lambda y, t : self.dx(*y, action)
        return odeint(f, state, [0, self.step_size])[1, :]
    
    def step(self, action):
        self.u = [-self.u_max, self.u_max, np.clip(-self.K@self.x, -self.u_max, self.u_max)[0]][action]
        self.x = self.transition(self.x, self.u)
        self.x[0] = np.remainder(self.x[0] + np.pi, 2*np.pi) - np.pi
        return self.state(), self.reward(), self.terminal()
    
    def render(self, cancel = False):
        try:
            self.anim.event_source.stop()
        except:
            pass
        fig = plt.figure()
        ax = plt.axes(xlim=(-2, 2), ylim=(-1.1, 1.1), aspect='equal')
        line, = ax.plot([], [], lw=2, marker='o', markersize=6)
        animate = lambda args: line.set_data([0, np.sin(self.x[0])], [0, np.cos(self.x[0])])
        self.anim = animation.FuncAnimation(fig, animate, interval=200)
        plt.show()
        return self.anim