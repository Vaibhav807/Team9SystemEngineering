# Controller Simulation GUI

This project provides a GUI for simulating and visualizing the performance of different controllers on a dynamic system. Our GUI supports four types of controllers:

- **PID**
- **Pole Placement**
- **LQR**
- **NMPC**

## Features

- **Controller Selection:**  
  Choose between PID, Pole Placement, LQR, and NMPC controllers using radio buttons.

- **Parameter Adjustment:**  
  For the PID controller, you can adjust the gains (Kp, Ki, Kd) via sliders.

- **Disturbance Injection:**  
  A disturbance can be applied to the system using a toggle switch and a slider. The slider range is set from -40 to 40, allowing you to simulate disturbances acting in either the left or right direction. A purple arrow representing the direction of disturbance appears above the cart.

- **Noise Injection:**  
  Noise can be added to the system via a toggle switch and a slider for noise amplitude.

- **Real-Time Visualization:**  
  The GUI displays:
  - The pole angle vs. time.
  - The cart position vs. time.
  - The control force vs. time.
  - Additional plots for disturbance and noise.
  - An animation of the cart-pendulum system with a visual disturbance arrow.

## Installation

Ensure you have the required dependencies installed:

- Python 3.x
- NumPy
- Matplotlib

You also need to have the simulation modules (e.g., `PIDSimulator`, `PolePlacementSimulator`, `LQRSimulator`, `NMPCSimulator`) available in your Python path.

## Usage

1. Run the Python script called "ControllerGUI".
2. Use the radio buttons to select the desired controller.
3. Adjust PID parameters using the Kp, Ki, and Kd sliders(if the pid controller is choosed) .
4. Toggle the disturbance switch and set the amplitude (use negative values for leftward disturbances and positive for rightward disturbances).
5. Optionally, enable noise and adjust its amplitude.
6. Click **Start** to begin the simulation. The system state and plots will update in real time.
7. Click **End** to stop the simulation and display the final results.

## Code Structure

- **Controller Selection and Parameter Controls:**  
  The left panel of the GUI contains radio buttons for selecting the controller and sliders for adjusting PID gains, disturbance amplitude, and noise amplitude.
  
- **Animation and Plotting:**  
  The right side of the GUI displays the dynamic simulation plots (pole angle, cart position, control force) and the cart-pendulum animation. The disturbance is visualized with an arrow that appears above the cart.

- **Simulation Loop:**  
  The simulation loop runs in batches, updating both the system's state and the visual components (plots and animation) continuously until the simulation time is reached or the simulation is manually stopped.

## Customization

You can adjust parameters such as simulation time (`T_total`), time step (`dt_sim`), and batch sizes directly in the code. The arrow scaling factor in the `update_pendulum` function can also be modified if you wish to change the visual representation of the disturbance.
