import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import RadioButtons, Button, Slider, CheckButtons
import matplotlib.patches as patches
import math
from PID import PIDSimulator
from PolePlacement import PolePlacementSimulator
from NMPC import NMPCSimulator
from LQR import LQRSimulator

sim_running = False
disturb_enabled = False
noise_enabled = False
simulator_global = None

controller_modes = {
    "PID": PIDSimulator,
    "PolePlacement": PolePlacementSimulator,
    "LQR": LQRSimulator,
    "NMPC": NMPCSimulator
}
current_controller = PIDSimulator

arrow_disturb = None

fig = plt.figure(figsize=(14, 8))
plt.subplots_adjust(left=0.05, bottom=0.05, right=0.95, top=0.92)
fig.suptitle("Controller GUI", fontsize=14)

rax = plt.axes([0.06, 0.62, 0.12, 0.15])
radio = RadioButtons(rax, ('PID', 'PolePlacement', 'LQR', 'NMPC'))
def radio_func(label):
    global current_controller
    current_controller = controller_modes[label]
    print("Selected controller:", label)
radio.on_clicked(radio_func)

ax_kp = plt.axes([0.06, 0.52, 0.12, 0.03])
ax_ki = plt.axes([0.06, 0.47, 0.12, 0.03])
ax_kd = plt.axes([0.06, 0.42, 0.12, 0.03])
slider_kp = Slider(ax_kp, 'Kp', 0.0, 70.0, valinit=50)
slider_ki = Slider(ax_ki, 'Ki', 0.0, 30.0, valinit=15)
slider_kd = Slider(ax_kd, 'Kd', 0.0, 10.0, valinit=0.1)

rax_disturb = plt.axes([0.06, 0.35, 0.12, 0.03])
check_disturb = CheckButtons(rax_disturb, ['Disturbance'], [False])
def disturb_func(label):
    global disturb_enabled
    disturb_enabled = not disturb_enabled
check_disturb.on_clicked(disturb_func)

ax_amp = plt.axes([0.06, 0.30, 0.12, 0.03])
slider_amp = Slider(ax_amp, 'Amp', -40.0, 40.0, valinit=0.0)

rax_noise = plt.axes([0.06, 0.23, 0.12, 0.03])
check_noise = CheckButtons(rax_noise, ['Noise'], [False])
def noise_func(label):
    global noise_enabled
    noise_enabled = not noise_enabled
check_noise.on_clicked(noise_func)

ax_noise = plt.axes([0.06, 0.18, 0.12, 0.03])
slider_noise = Slider(ax_noise, 'Noise Amp', 0.0, 0.1, valinit=0.0)

start_ax = plt.axes([0.06, 0.10, 0.05, 0.05])
start_button = Button(start_ax, 'Start')
end_ax = plt.axes([0.13, 0.10, 0.05, 0.05])
end_button = Button(end_ax, 'End')

ax1 = plt.axes([0.25, 0.63, 0.45, 0.25])
ax2 = plt.axes([0.25, 0.33, 0.45, 0.25])
ax3 = plt.axes([0.25, 0.03, 0.45, 0.25])

ax_pendulum = plt.axes([0.73, 0.65, 0.22, 0.25])
ax_pendulum.set_xlim([-0.8, 0.8])
ax_pendulum.set_ylim([-0.1, 0.8])
ax_pendulum.set_xticks([])
ax_pendulum.set_yticks([])
ax_pendulum.set_title("Animation")
cart_width = 0.15
cart_height = 0.07
cart_body = patches.Rectangle((0 - cart_width/2, 0), cart_width, cart_height, facecolor='gray', edgecolor='black')
cart_body.set_visible(False)
ax_pendulum.add_patch(cart_body)
pole, = ax_pendulum.plot([], [], 'r-', lw=3)
pendulum_tip, = ax_pendulum.plot([], [], 'bo', markersize=8)
track_line, = ax_pendulum.plot([-0.8, 0.8], [0, 0], 'k--', lw=1)

ax_disturb = plt.axes([0.73, 0.36, 0.22, 0.25])
ax_disturb.set_title("Disturbance (N)")
line_disturb, = ax_disturb.plot([], [], 'm-', lw=2)
ax_disturb.set_xlim(0, 10)
ax_disturb.set_ylim(-20, 20)

ax_noise_plot = plt.axes([0.73, 0.06, 0.22, 0.25])
ax_noise_plot.set_title("Noise (rad)")
line_noise, = ax_noise_plot.plot([], [], 'c-', lw=2)
ax_noise_plot.set_xlim(0, 10)
ax_noise_plot.set_ylim(-1, 1)

T_total = 50.0
disturb_data = []
noise_data = []

def start_simulation(event):
    global sim_running, disturb_data, noise_data, simulator_global
    print("Start Simulation button clicked!")
    sim_running = True
    current_label = radio.value_selected
    if current_label == "PID":
        dt_sim = 0.01
        batch_size = 10
        simulator = current_controller(T_total=T_total, dt=dt_sim, Kp=slider_kp.val, Ki=slider_ki.val, Kd=slider_kd.val)
    elif current_label == "PolePlacement":
        dt_sim = 0.001
        batch_size = 50
        simulator = current_controller(T_total=T_total, dt=dt_sim)
    elif current_label == "LQR":
        dt_sim = 0.001
        batch_size = 30
        simulator = current_controller(T_total=T_total, dt=dt_sim)
    elif current_label == "NMPC":
        dt_sim = 0.01
        batch_size = 10
        simulator = current_controller(T_total=T_total, dt=dt_sim)
    else:
        print("Selected controller not implemented.")
        return
    simulator_global = simulator
    steps = int(T_total / dt_sim)
    cart_body.set_visible(True)
    ax1.clear(); ax2.clear(); ax3.clear(); ax_disturb.clear(); ax_noise_plot.clear()
    line1, = ax1.plot([], [], label="θ (rad)")
    ax1.axhline(0, color='r', linestyle='--', label="Desired 0")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Pole Angle (rad)")
    ax1.legend()
    line2, = ax2.plot([], [], label="x (m)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Cart Position (m)")
    ax2.legend()
    line3, = ax3.plot([], [], label="Control Force F (N)")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Force F (N)")
    ax3.legend()
    line_d, = ax_disturb.plot([], [], 'm-', lw=2, label="Disturbance (N)")
    ax_disturb.set_xlabel("Time (s)")
    ax_disturb.set_ylabel("Force (N)")
    ax_disturb.legend()
    line_n, = ax_noise_plot.plot([], [], 'c-', lw=2, label="Noise (rad)")
    ax_noise_plot.set_xlabel("Time (s)")
    ax_noise_plot.set_ylabel("Noise (rad)")
    ax_noise_plot.legend()
    disturb_data = []
    noise_data = []
    num_batches = steps // batch_size
    for i in range(num_batches):
        if not sim_running:
            print("Simulation ended early.")
            break
        if current_label == "PID":
            simulator.set_pid_parameters(slider_kp.val, slider_ki.val, slider_kd.val)
        if disturb_enabled:
            disturbance = slider_amp.val  
        else:
            disturbance = 0.0
        if noise_enabled:
            noise = np.random.uniform(-0.05, 0.05)
            noise = slider_noise.val * noise
        else:
            noise = 0.0
        simulator.set_disturbance(disturbance)
        simulator.set_noise(noise)
        disturb_data.extend([disturbance] * batch_size)
        noise_data.extend([noise] * batch_size)
        simulator.step_batch(batch_size)
        line1.set_data(simulator.time_data, simulator.theta_data)
        line2.set_data(simulator.time_data, simulator.x_data)
        line3.set_data(simulator.time_data, simulator.F_data)
        line_d.set_data(simulator.time_data, disturb_data)
        line_n.set_data(simulator.time_data, noise_data)
        ax1.relim(); ax1.autoscale_view()
        ax2.relim(); ax2.autoscale_view()
        ax3.relim(); ax3.autoscale_view()
        ax_disturb.relim(); ax_disturb.autoscale_view()
        ax_noise_plot.relim(); ax_noise_plot.autoscale_view()
        update_pendulum(simulator.state[0], simulator.state[2], disturbance)
        plt.pause(0.001)
    if sim_running and simulator.current_step < simulator.steps:
        simulator.step_batch(simulator.steps - simulator.current_step)
        line1.set_data(simulator.time_data, simulator.theta_data)
        line2.set_data(simulator.time_data, simulator.x_data)
        line3.set_data(simulator.time_data, simulator.F_data)
        line_d.set_data(simulator.time_data, disturb_data)
        line_n.set_data(simulator.time_data, noise_data)
        ax1.relim(); ax1.autoscale_view()
        ax2.relim(); ax2.autoscale_view()
        ax3.relim(); ax3.autoscale_view()
        ax_disturb.relim(); ax_disturb.autoscale_view()
        ax_noise_plot.relim(); ax_noise_plot.autoscale_view()
        plt.pause(0.001)
    plt.ioff()

start_button.on_clicked(start_simulation)

def end_simulation(event):
    global sim_running
    sim_running = False
    print("End Simulation clicked.")
    plot_results()
end_button.on_clicked(end_simulation)

def update_pendulum(cart_x, theta, disturbance=0.0):
    cart_body.set_xy((cart_x - cart_width/2, 0))
    current_xlim = ax_pendulum.get_xlim()
    half_width = (current_xlim[1] - current_xlim[0]) / 2.0
    if cart_x - half_width < current_xlim[0] or cart_x + half_width > current_xlim[1]:
        ax_pendulum.set_xlim(cart_x - half_width, cart_x + half_width)
    pivot_x = cart_x
    pivot_y = cart_height
    L = 0.05
    bob_x = pivot_x + L * math.sin(theta)
    bob_y = pivot_y + L * math.cos(theta)
    pole.set_data([pivot_x, bob_x], [pivot_y, bob_y])
    pendulum_tip.set_data(bob_x, bob_y)
    global arrow_disturb
    if arrow_disturb is not None:
        try:
            arrow_disturb.remove()
        except Exception:
            pass
    if abs(disturbance) > 1e-6:
        scale = 0.015
        arrow_dx = disturbance * scale
        arrow_start_x = cart_x
        arrow_start_y = cart_height + 0.02
        arrow = patches.FancyArrow(arrow_start_x, arrow_start_y, arrow_dx, 0, width=0.01, head_width=0.05, head_length=0.03, color='purple')
        ax_pendulum.add_patch(arrow)
        arrow_disturb = arrow
    else:
        arrow_disturb = None
    plt.draw()

def plot_results():
    global simulator_global
    if simulator_global is None:
        print("No simulation data to plot.")
        return
    fig_theta = plt.figure()
    plt.plot(simulator_global.time_data, simulator_global.theta_data, 'b-', label="θ (rad)")
    plt.axhline(0, color='r', linestyle='--', label="Desired 0")
    plt.xlabel("Time (s)")
    plt.ylabel("Pole Angle (rad)")
    plt.legend()
    plt.title("Pole Angle vs Time")
    plt.show()
    fig_x = plt.figure()
    plt.plot(simulator_global.time_data, simulator_global.x_data, 'g-', label="x (m)")
    plt.xlabel("Time (s)")
    plt.ylabel("Cart Position (m)")
    plt.legend()
    plt.title("Cart Position vs Time")
    plt.show()
    fig_F = plt.figure()
    plt.plot(simulator_global.time_data, simulator_global.F_data, 'm-', label="Control Force F (N)")
    plt.xlabel("Time (s)")
    plt.ylabel("Force (N)")
    plt.legend()
    plt.title("Control Force vs Time")
    plt.show()

plt.show()
