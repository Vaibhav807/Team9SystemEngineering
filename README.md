# Simulation Controller 

Our simulation provides a GUI for visualizing the performance of different controllers on a dynamic system. Our GUI supports four types of controllers:

- **PID**
- **Pole Placement**
- **LQR**
- **NMPC**

## Features

- **Controller Selection:**  
  Choose between PID, Pole Placement, LQR, and NMPC by using radio buttons.

- **Parameter Adjustment:**  
  For the PID controller, you can adjust the gains (Kp, Ki, Kd) via sliders.

- **Disturbance Injection:**  
  A disturbance can be applied to the system using a toggle switch and a slider. The slider range is ranged from -40 to 40, allowing you to simulate disturbances acting in either the left or right direction.
  
- **Noise Injection:**  
  Noise can be added to the system by using a toggle switch and a slider for noise amplitude.

- **Real-Time Visualization:**  
  The GUI displays:
  - The pole angle vs. time.
  - The cart position vs. time.
  - The control force vs. time.
  - Additional plots for disturbance and noise.
  - An animation of the cart-pendulum system with a visual disturbance arrow.

## Installation

Ensure you have the required dependencies installed:

- Python 
- NumPy
- Matplotlib

You also need to have the simulation modules (e.g., `PIDSimulator`, `PolePlacementSimulator`, `LQRSimulator`, `NMPCSimulator`) available in your Python path.

## Usage

1. Run a Python script called "ControllerGUI".
2. Use the radio buttons to select the controller.
3. Adjust PID parameters using the Kp, Ki, and Kd sliders(if the pid controller is choosed) .
4. Toggle the disturbance switch and set the amplitude (use negative values for leftward disturbances and positive for rightward disturbances).
5. Optionally, enable noise and adjust its amplitude.
6. Click **Start** to begin the simulation. 
7. Click **End** to stop the simulation and show the final results.

## Customization

You can adjust parameters such as simulation time (`T_total`), time step (`dt_sim`), and batch sizes directly in the code.
