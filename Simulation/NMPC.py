import casadi as ca
import numpy as np

class NMPCSimulator:
    def __init__(self, T_total=5.0, dt=0.01, N=200):
        self.T_total = T_total
        self.dt = dt
        self.N = N
        
        self.M = 0.867
        self.m_tot = 0.05
        self.l_c = 0.46
        self.I_tot = 0.01058
        self.cart_damping = 0.6
        self.pivot_damping = 0.005
        self.g = 9.81
        
        self.n_states = 4
        self.n_controls = 1
        
        x = ca.SX.sym('x', self.n_states)
        u = ca.SX.sym('u', self.n_controls)
        
        A_mat = ca.vertcat(
            ca.horzcat(self.M + self.m_tot, - self.m_tot * self.l_c * ca.cos(x[2])),
            ca.horzcat(- self.m_tot * self.l_c * ca.cos(x[2]), self.m_tot * (self.l_c**2) + self.I_tot)
        )
        b_vec = ca.vertcat(
            u - self.m_tot * self.l_c * ca.sin(x[2]) * (x[3]**2) - self.cart_damping * x[1],
            self.m_tot * self.g * self.l_c * ca.sin(x[2]) - self.pivot_damping * x[3]
        )
        acc = ca.mtimes(ca.inv(A_mat), b_vec)
        f = ca.vertcat(x[1], acc[0], x[3], acc[1])
        x_next = x + self.dt * f
        self.f_d = ca.Function('f_d', [x, u], [x_next])
        
        self.Q = np.diag([1, 0.1, 5, 0.1])
        self.R = np.array([[0.01]])
        self.Qf = self.Q
        
        X = ca.SX.sym('X', self.n_states, self.N+1)
        U = ca.SX.sym('U', self.n_controls, self.N)
        X0_param = ca.SX.sym('X0', self.n_states)
        
        obj = 0
        g_nlp = []
        g_nlp.append(X[:, 0] - X0_param)
        for k in range(self.N):
            obj += self.dt * (ca.mtimes([X[:, k].T, self.Q, X[:, k]]) + ca.mtimes([U[:, k].T, self.R, U[:, k]]))
            x_next_k = self.f_d(X[:, k], U[:, k])
            g_nlp.append(X[:, k+1] - x_next_k)
        obj += self.dt * ca.mtimes([X[:, self.N].T, self.Qf, X[:, self.N]])
        
        g_nlp = ca.vertcat(*g_nlp)
        opt_variables = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        nlp_prob = {'f': obj, 'x': opt_variables, 'g': g_nlp, 'p': X0_param}
        opts = {"print_time": False, "ipopt": {"print_level": 0}}
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)
        
        self.steps = int(self.T_total / self.dt)
        self.current_step = 0
        self.state = np.array([0.0, 0.0, 0.1, 0.0])
        self.time_data = []
        self.x_data = []
        self.theta_data = []
        self.F_data = []
        self.t = 0.0
        self.disturbance = 0.0
        self.noise = 0.0

    def set_disturbance(self, disturbance):
        self.disturbance = disturbance

    def set_noise(self, noise):
        self.noise = noise

    def step(self):
        if self.current_step >= self.steps:
            return self.state
        sol = self.solver(p=self.state, lbg=0, ubg=0)
        sol_val = sol['x'].full().flatten()
        U_opt = sol_val[self.n_states*(self.N+1):]
        u0 = U_opt[0]
        current_control = u0 + self.disturbance
        new_state = self.f_d(self.state, [current_control]).full().flatten()
        new_state[2] = new_state[2] + self.noise
        self.state = new_state
        self.time_data.append(self.t)
        self.x_data.append(self.state[0])
        self.theta_data.append(self.state[2])
        self.F_data.append(current_control)
        self.t += self.dt
        self.current_step += 1
        return self.state

    def step_batch(self, batch_size):
        for _ in range(batch_size):
            if self.current_step >= self.steps:
                print("END")
                break
            self.step()
        return self.state

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    sim = NMPCSimulator(T_total=10.0, dt=0.01, N=100)
    for i in range(sim.steps):
        sim.step()
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
