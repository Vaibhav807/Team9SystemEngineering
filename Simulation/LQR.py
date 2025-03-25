import numpy as np
from scipy.linalg import solve_continuous_are

class LQRController:
    def __init__(self, A, B, Q, R):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        P = solve_continuous_are(A, B, Q, R)
        self.K = np.linalg.inv(R) @ B.T @ P

    def update(self, state):
        u = -self.K @ state
        return u.item() if np.isscalar(u) is False else u

class LQRSimulator:
    def __init__(self, T_total=10.0, dt=0.001, Q=None, R=None):
        self.M = 0.867
        self.m_tot = 0.05
        self.l_c = 0.46
        self.I_tot = 0.01058
        self.cart_damping = 0.3
        self.pivot_damping = 0.001
        self.g = 9.81
        
        self.dt = dt
        self.T_total = T_total
        self.steps = int(self.T_total / self.dt)
        self.current_step = 0
        
        self.state = np.array([0.0, 0, 0.1, 0])
        self.t = 0.0
        self.time_data = []
        self.x_data = []
        self.theta_data = []
        self.F_data = []
        self.disturbance = 0.0
        self.noise = 0.0

        a = self.M + self.m_tot
        b_val = - self.m_tot * self.l_c
        c = self.m_tot * self.l_c**2 + self.I_tot
        det = a * c - (self.m_tot * self.l_c)**2

        A_lin = np.array([
            [0, 1, 0, 0],
            [0, - (c * self.cart_damping / det), (self.m_tot**2 * self.l_c**2 * self.g / det), - (self.m_tot * self.l_c * self.pivot_damping / det)],
            [0, 0, 0, 1],
            [0, - (self.m_tot * self.l_c * self.cart_damping / det), (a * self.m_tot * self.g * self.l_c / det), - (a * self.pivot_damping / det)]
        ])
        B_lin = np.array([
            [0],
            [c/det],
            [0],
            [self.m_tot * self.l_c / det]
        ])

        if Q is None:
            Q = np.diag([1, 1, 5, 1])
        if R is None:
            R = np.array([[0.01]])

        self.lqr = LQRController(A_lin, B_lin, Q, R)

    def set_disturbance(self, disturbance):
        self.disturbance = disturbance

    def set_noise(self, noise):
        self.noise = noise

    def step(self):
        if self.current_step >= self.steps:
            return self.state

        x1, x2, x3, x4 = self.state
        F = self.lqr.update(self.state)
        F_total = F + self.disturbance
        F_total = np.clip(F_total, -1000, 1000)
        
        A = np.array([
            [self.M + self.m_tot, - self.m_tot * self.l_c * np.cos(x3)],
            [- self.m_tot * self.l_c * np.cos(x3), self.m_tot * self.l_c**2 + self.I_tot]
        ])
        b = np.array([
            F_total - self.m_tot * self.l_c * np.sin(x3) * (x4**2) - self.cart_damping * x2,
            self.m_tot * self.g * self.l_c * np.sin(x3) - self.pivot_damping * x4
        ])
        x_ddot, phi_ddot = np.linalg.solve(A, b)
        
        x1_new = x1 + x2 * self.dt
        x2_new = x2 + x_ddot * self.dt
        x3_new = x3 + x4 * self.dt + self.noise
        x4_new = x4 + phi_ddot * self.dt
        self.state = np.array([x1_new, x2_new, x3_new, x4_new])
        
        self.t += self.dt
        self.current_step += 1
        self.time_data.append(self.t)
        self.x_data.append(x1_new)
        self.theta_data.append(x3_new)
        self.F_data.append(F_total)
        
        return self.state

    def step_batch(self, batch_size):
        for i in range(batch_size):
            if self.current_step >= self.steps:
                print("END")
                break
            self.step()
        return self.state

if __name__ == '__main__':
    sim = LQRSimulator(T_total=15.0, dt=0.001)
    for i in range(sim.steps):
        state = sim.step()
    import matplotlib.pyplot as plt
    plt.figure()
    plt.subplot(2,1,1)
    plt.plot(sim.time_data, sim.x_data)
    plt.xlabel("Time (s)")
    plt.ylabel("Cart Position (m)")
    plt.subplot(2,1,2)
    plt.plot(sim.time_data, sim.theta_data)
    plt.xlabel("Time (s)")
    plt.ylabel("Pendulum Angle (rad)")
    plt.tight_layout()
    plt.show()
