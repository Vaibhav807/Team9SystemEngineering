import numpy as np

class PIDSimulator:
    class PIDController:
        def __init__(self, Kp, Ki, Kd, setpoint=0):
            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd
            self.setpoint = setpoint
            self._integral = 0.0
            self._last_error = 0.0

        def update(self, measurement, dt):
            error = self.setpoint - measurement
            self._integral += error * dt
            derivative = (error - self._last_error) / dt
            output = self.Kp * error + self.Ki * self._integral + self.Kd * derivative
            self._last_error = error
            return output

        def set_parameters(self, Kp, Ki, Kd):
            self.Kp = Kp
            self.Ki = Ki
            self.Kd = Kd

    def __init__(self, T_total=100.0, dt=0.01, Kp=0, Ki=0, Kd=0):
        self.M = 0.867
        self.m_tot = 0.05
        self.l_c = 0.46
        self.I_tot = 0.01058
        self.cart_damping = 0.6
        self.pivot_damping = 0.005
        self.g = 9.81
        self.pid = PIDSimulator.PIDController(Kp, Ki, Kd, setpoint=0.0)
        self.output_limits = (-100, 100)
        self.dt = dt
        self.T_total = T_total
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

    def reset(self):
        self.current_step = 0
        self.t = 0.0
        self.state = np.array([0.0, 0.0, 0.1, 0.0])
        self.time_data = []
        self.x_data = []
        self.theta_data = []
        self.F_data = []
        self.pid._integral = 0.0
        self.pid._last_error = 0.0

    def set_pid_parameters(self, Kp, Ki, Kd):
        self.pid.set_parameters(Kp, Ki, Kd)

    def set_disturbance(self, disturbance):
        self.disturbance = disturbance

    def set_noise(self, noise):
        self.noise = noise

    def step(self):
        if self.current_step >= self.steps:
            return self.state
        x1, x2, x3, x4 = self.state
        F = self.pid.update(x3, self.dt)
        F_total = F + self.disturbance
        A = np.array([
            [self.M + self.m_tot, - self.m_tot * self.l_c * np.cos(x3)],
            [- self.m_tot * self.l_c * np.cos(x3), self.m_tot * (self.l_c**2) + self.I_tot]
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
        self.time_data.append(self.t)
        self.x_data.append(x1_new)
        self.theta_data.append(x3_new)
        self.F_data.append(F_total)
        self.t += self.dt
        self.current_step += 1
        return self.state

    def step_batch(self, batch_size):
        for i in range(batch_size):
            if self.current_step >= self.steps:
                print("END")
                break
            self.step()
        return self.state

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    sim = PIDSimulator(T_total=50.0, dt=0.001, Kp=100, Ki=0, Kd=0)
    for i in range(sim.steps):
        sim.step()
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(sim.time_data, sim.x_data)
    plt.xlabel("Time (s)")
    plt.ylabel("Cart Position (m)")
    plt.subplot(2, 1, 2)
    plt.plot(sim.time_data, sim.theta_data)
    plt.xlabel("Time (s)")
    plt.ylabel("Pendulum Angle (rad)")
    plt.tight_layout()
    plt.show()
