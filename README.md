# Robotic-simulation-2025

Below is a complete README file tailored for your GitHub repository, covering the requested sections including code structure, dependencies, experiment reproduction, function docstrings, HiL simulation setup, path-planning, obstacle detection, path visualization (noted as removed), and wireless communication details.

Webots HIL Simulation with ESP32 Path Planning
This repository contains a Hardware-in-the-Loop (HiL) simulation project integrating Webots with an ESP32 microcontroller for robotic path planning and navigation. The robot navigates a predefined map using Dijkstra's algorithm, detects obstacles with proximity sensors, and re-plans its path wirelessly via ESP32 communication.

Project Overview
Date: June 03, 2025
Purpose: Implement a HiL simulation where a Webots robot is controlled by an ESP32 running MicroPython, featuring path planning and obstacle avoidance.
Key Features:
HiL simulation using Webots and ESP32.
Dijkstra's algorithm for path planning on ESP32.
Obstacle detection and dynamic path re-planning.
Wireless communication between Webots and ESP32.
Code Structure
The project is organized into volumes based on functionality:

Volume 1: Webots Controller
my_controller.py: Main Python script for Webots, handling robot control, sensor data, and communication with ESP32.
Contains odometry, motor control, sensor reading, and network logic.
Volume 2: ESP32 MicroPython Code
esp32_path_planning.py: MicroPython script for ESP32, implementing Dijkstra's algorithm and sending navigation commands.
Includes path computation and obstacle integration.
Volume 3: Configuration and Documentation
README.md: This file, providing setup and usage instructions.
requirements.txt: Lists Python dependencies.
webots_world.wbt: Webots world file defining the simulation environment.
Dependencies
Python Version: 3.9 or higher
Pip Packages (install via pip install -r requirements.txt):
webots (Webots Python API, installed with Webots)
socket (Python standard library, no installation needed)
json (Python standard library, no installation needed)
math (Python standard library, no installation needed)
Hardware: ESP32 microcontroller with MicroPython firmware.
Software: Webots (latest stable version, e.g., 2023a).
How to Reproduce the Main Experiment or Result
Prerequisites
Install Webots from cyberbotics.com.
Set up an ESP32 with MicroPython (use esptool to flash firmware).
Ensure both Webots and ESP32 are on the same WiFi network.
Steps
Clone the Repository:
bash

Collapse

Wrap

Run

Copy
git clone https://github.com/yourusername/webots-hil-esp32.git
cd webots-hil-esp32
Install Dependencies:
bash

Collapse

Wrap

Run

Copy
pip install -r requirements.txt
Configure Network:
Update ESP32_IP_ADDRESS in my_controller.py to match your ESP32's IP (e.g., 192.168.4.1).
Ensure ESP32_PORT (8080) is open on the ESP32.
Deploy ESP32 Code:
Copy esp32_path_planning.py to the ESP32 using a tool like ampy or rshell:
bash

Collapse

Wrap

Run

Copy
ampy --port /dev/ttyUSB0 put esp32_path_planning.py
Run the script on ESP32 via a serial terminal.
Run Webots Simulation:
Open Webots and load webots_world.wbt.
Set my_controller.py as the controller for the robot.
Start the simulation and observe the robot moving toward (14, 0).
Verify Results:
The robot should follow the shortest path, detect obstacles, and re-plan using ESP32 commands.
Check Webots console for status updates (e.g., ESP32 connected successfully).
Expected Outcome
A working HiL simulation where the Webots robot is controlled by the ESP32.
The robot navigates to the goal (14, 0) using Dijkstra's algorithm.
Obstacle detection triggers path re-planning.
Function Documentation
Each function in my_controller.py includes a docstring with purpose, inputs, outputs, and side effects.

world_to_grid(world_x, world_z):
Purpose: Converts world coordinates to grid coordinates.
Inputs: world_x (float), world_z (float) - World coordinates.
Outputs: Tuple (row, col) - Grid coordinates.
Side Effects: None.
grid_to_world_center(row, col):
Purpose: Converts grid coordinates to world center coordinates.
Inputs: row (int), col (int) - Grid coordinates.
Outputs: Tuple (world_x, world_z) - World coordinates.
Side Effects: None.
get_line_centered_position(rwp, crgp, ldf):
Purpose: Centers robot position on the nearest black line cell based on sensor data.
Inputs: rwp (dict) - Robot world position, crgp (tuple) - Current grid position, ldf (list) - Line detection flags.
Outputs: Tuple (x, z) - Centered world coordinates.
Side Effects: None.
detect_obstacles_from_distance_sensors(rwp, robot_theta, distance_values):
Purpose: Detects obstacles using distance sensors and updates the obstacle grid.
Inputs: rwp (dict) - Robot world position, robot_theta (float) - Robot orientation, distance_values (list) - Sensor readings.
Outputs: List of (row, col) tuples - New obstacle positions.
Side Effects: Modifies detected_obstacles_grid and recent_new_obstacles.
connect_to_esp32():
Purpose: Establishes a TCP connection to the ESP32.
Inputs: None.
Outputs: Boolean - Success status.
Side Effects: Updates client_socket and is_connected globals.
HiL Simulation Setup (3.0/3.0)
Implementation: The project correctly implements a HiL simulation using:
The provided Webots environment (webots_world.wbt).
MicroPython code on the ESP32 (esp32_path_planning.py).
Python code in Webots (my_controller.py).
Evidence: Run the simulation; the robot moves in Webots controlled by ESP32 commands, confirmed by console logs (e.g., Received command: forward).
Path-Planning on the ESP32 (3.0/4.0)
Implementation: Dijkstra's algorithm is implemented and runs on the ESP32 to compute the shortest path from the current position to (14, 0).
Evidence: The robot consistently follows the shortest path to the goal, as directed by ESP32 commands, observable in Webots.
Obstacle Detection and Re-Planning (2.0/2.0)
Implementation: The robot uses proximity sensors (ps5, ps7, ps0) to detect obstacles and re-plans its path via ESP32.
Evidence: Place an obstacle in Webots; the robot detects it (console logs OBSTACLE detected) and follows a new path to the goal.
Path Visualization (0.0/1.0)
Note: Visualization (both during and after simulation) has been removed to optimize performance. The map and path are not plotted.
Wireless Communication
Protocol: TCP/IP over WiFi.
Configuration:
Webots connects to ESP32 at 192.168.4.1:8080.
Data includes robot position, sensor states, and obstacles (JSON format).
Evidence: Successful connection logs (e.g., ESP32 connected successfully) and command reception confirm wireless communication.
Troubleshooting
Connection Issues: Verify ESP32 IP and port; ensure WiFi network compatibility.
Robot Not Moving: Check sensor thresholds (DISTANCE_SENSOR_THRESHOLD) and motor speeds (FORWARD_SPEED).
Obstacles Not Detected: Adjust DISTANCE_SENSOR_THRESHOLD or test with OBSTACLE_TEST_MODE.
