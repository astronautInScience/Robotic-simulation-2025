---

# Webots HIL Simulation with ESP32 Path Planning 🌐🤖🔧

This repository contains a Hardware-in-the-Loop (HiL) simulation project integrating Webots with an ESP32 microcontroller for robotic path planning and navigation. The robot navigates a predefined map using the A\* (A-star) algorithm, detects obstacles with proximity sensors, and re-plans its path wirelessly via ESP32 communication. 🚀📡🧠

## Project Overview 🎯📘🛠️

* **Purpose**: Implement a HiL simulation where a Webots robot is controlled by an ESP32 running MicroPython, featuring path planning and obstacle avoidance.
* **Key Features**:

  * HiL simulation using Webots and ESP32.
  * A\* algorithm for path planning on ESP32.
  * Obstacle detection and dynamic path re-planning.
  * Wireless communication between Webots and ESP32.

## Code Structure 🗂️📁💡

The project is organized into modules based on functionality:

### Webots Controller 🕹️📷📨

* `my_controller.py`: Main Python script for Webots, handling robot control, sensor data, and communication with ESP32.

  * Includes odometry, motor control, sensor reading, obstacle mapping, and socket-based networking.

### ESP32 MicroPython Code 💾📟💡

* `esp32_path_planning.py`: MicroPython script for ESP32, implementing the A\* algorithm and sending navigation commands.

  * Performs path computation and dynamic re-planning.

### Configuration and Documentation 📄⚙️📝

* `README.md`: This file, providing setup and usage instructions.
* `requirements.txt`: Lists Python dependencies.
* `webots_world.wbt`: Webots world file defining the simulation environment.

## Dependencies 🧩📦🖥️

* **Python Version**: 3.9 or higher
* **Pip Packages** (install via `pip install -r requirements.txt`):

  * `webots` (comes with Webots)
  * `socket`, `json`, `math` (Python standard libraries)
* **Hardware**: ESP32 microcontroller flashed with MicroPython
* **Software**: Webots (latest stable version, e.g., 2023a)

## How to Reproduce the Main Experiment or Result 🔁⚗️🧪

### Prerequisites 🔧📲💻

1. Install Webots from [cyberbotics.com](https://cyberbotics.com/).
2. Flash MicroPython firmware on the ESP32 using [esptool](https://github.com/espressif/esptool).
3. Ensure both Webots and ESP32 are on the same WiFi network.

### Steps 🪜🧭🧰

1. **Clone the Repository**:

   ```bash
   git clone https://github.com/yourusername/webots-hil-esp32.git
   cd webots-hil-esp32
   ```

2. **Install Dependencies**:

   ```bash
   pip install -r requirements.txt
   ```

3. **Configure Network**:

   * Update `ESP32_IP_ADDRESS` in `my_controller.py` to your ESP32 IP (e.g., `192.168.4.1`).
   * Ensure `ESP32_PORT` (e.g., 8080) matches the ESP32 script.

4. **Deploy ESP32 Code**:

   ```bash
   ampy --port /dev/ttyUSB0 put esp32_path_planning.py
   ```

   * Then connect to the serial port and run the script.

5. **Run Webots Simulation**:

   * Open Webots, load `webots_world.wbt`.
   * Set `my_controller.py` as the robot controller.
   * Start the simulation.

6. **Verify Results**:

   * Observe robot following path to `(14, 0)`.
   * Monitor Webots console for logs like `ESP32 connected successfully` and `Received command: forward`.

### Expected Outcome 🎯🤖🟢

* Robot successfully follows a path planned on the ESP32.
* Detects and avoids obstacles using proximity sensors.
* Path is dynamically re-planned on obstacle detection.

## Function Documentation 📚🔍📄

Sample of documented functions from `my_controller.py`:

* `world_to_grid()`:

  * Converts Webots world coordinates to grid coordinates.

* `grid_to_world()`:

  * Converts grid coordinates to Webots world center position.

* `read_distance_sensors()`:

  * Reads distance sensors and updates the obstacle grid.

* `connect_to_esp32()`:

  * Establishes TCP connection to ESP32.

## HiL Simulation Setup ⚙️🧪🤝

* **Implementation**:

  * Combines Webots simulation environment with ESP32 controller for real-time interaction.
  * Pathfinding and decision-making handled on embedded system.

* **Evidence**:

  * Webots robot follows external commands from ESP32.
  * Console shows real-time responses and obstacle updates.

## Path-Planning on the ESP32 🧠🗺️🚗

### Algorithm Used: A\* (A-star) 📌📐📊

A\* is an optimal path planning algorithm that combines Dijkstra’s shortest path search with a heuristic (Manhattan or Euclidean). It is defined by:

$f(n) = g(n) + h(n)$

Where:

* $g(n)$: Cost from start to node $n$
* $h(n)$: Heuristic cost estimate to goal

### Key Functions ⚙️🧮📘

1. **AStarPriorityQueue**:

   * Custom queue to maintain and retrieve nodes based on `f_score`.

2. **Heuristics**:

   * `manhattan_distance()`: $|x_1 - x_2| + |y_1 - y_2|$
   * `euclidean_distance()`: $\sqrt{(x_1 - x_2)^2 + (y_1 - y_2)^2}$

3. **get\_valid\_neighbors(r, c, rows, cols, grid)**:

   * Identifies 4-directional (up, down, left, right) valid neighbors.

4. **a\_star(grid, start\_node, end\_node, heuristic='manhattan')**:

   * Core A\* pathfinding function implemented on ESP32.

### Performance 🚀📈⚡

* Algorithm executes efficiently on ESP32.
* Capable of re-planning within milliseconds when obstacles appear.

## Obstacle Detection and Re-Planning 🚧📡♻️

* Robot detects obstacles via proximity sensors (`ps5`, `ps7`, `ps0`).
* Sends new obstacle data to ESP32.
* ESP32 updates its map and re-plans the path.
* Webots console confirms detection (`OBSTACLE detected`).

## Path Visualization 🖼️👣🧵

* Path visualization was disabled to optimize performance.
* Focus is on real-time execution and response.

---
