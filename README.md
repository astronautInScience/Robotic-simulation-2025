---

# Webots HiL Simulation with ESP32 Dijkstra Path Planning

This repository contains a Hardware-in-the-Loop (HiL) simulation system where a Webots-based e-puck robot receives navigation commands from an ESP32 microcontroller running MicroPython. The ESP32 executes Dijkstra's algorithm for path planning and communicates wirelessly with Webots.

## Project Overview

* **Purpose**: To demonstrate HiL simulation with path planning computed externally on an ESP32 and executed by a robot in Webots.
* **Key Features**:

  * Real-time serial communication between Webots and ESP32.
  * Dijkstra's algorithm implemented in MicroPython.
  * PID control and wall-following in Webots for navigation.
  * Modular code structure separating ESP32 logic from Webots control.

## Code Structure

### Webots Controller

* `my_controller.py`: Controls the e-puck robot in Webots based on path input from ESP32. Features:

  * Serial communication interface (USB/Thonny-compatible).
  * PID controller for movement.
  * Odometry and wheel encoder updates.
  * State machine logic for executing paths.

### ESP32 MicroPython Code

* `esp32_dijkstra.py`: Implements Dijkstra's algorithm to calculate the shortest path between nodes.

  * Upload using Thonny IDE via USB.
  * Sends computed path (as a list of nodes) to Webots.
  * Also controls program start/stop.

### Additional Files

* `webots_maze.wbt`: Webots world file including a maze environment with nodes.
* `README.md`: Project documentation and setup guide.

## Dependencies

* **Python Version**: 3.8+
* **Pip Packages**:

  * `pyserial` (for USB communication)
  * `math`, `json`, `time` (standard libraries)
* **Microcontroller**: ESP32 with MicroPython firmware.
* **Simulation**: Webots R2023a or newer.

## Getting Started

### Prerequisites

1. Install Webots from [Cyberbotics](https://cyberbotics.com/).
2. Flash ESP32 with MicroPython using [esptool](https://github.com/espressif/esptool).
3. Install Thonny for code uploading.

### Steps

1. **Clone the Repository**:

   ```bash
   git clone https://github.com/yourusername/webots-esp32-hil
   cd webots-esp32-hil
   ```

2. **Install Python Dependencies**:

   ```bash
   pip install pyserial
   ```

3. **Upload ESP32 Code**:

   * Open `esp32_dijkstra.py` in Thonny.
   * Connect ESP32 and upload the script.
   * Press the ESP32 button to start execution.

4. **Run Webots Simulation**:

   * Open `webots_maze.wbt` in Webots.
   * Assign `my_controller.py` to the e-puck robot.
   * Start simulation to observe robot path execution.

### Expected Outcome

* ESP32 sends shortest path node list to Webots.
* Webots robot follows the path, using PID control.
* Robot adjusts trajectory and avoids collisions using wall-following logic.

## Function Documentation

* \`\`:

  * **Purpose**: Computes shortest path from `start` to `end` in a node graph.
  * **Inputs**: `graph` (dict), `start` (str), `end` (str).
  * **Outputs**: List of node names representing shortest path.

* \`\` (Webots):

  * **Purpose**: Reads path commands from ESP32 via USB.
  * **Output**: List of node indices.

* \`\` (Webots):

  * **Purpose**: Executes movement to a specified node using PID and wheel control.

* \`\`:

  * **Purpose**: Updates robot position estimate using wheel encoders.

* \`\`:

  * **Purpose**: Uses proximity sensors to stay aligned with maze walls.

## HiL Integration

* **Design**: Path planning is fully offloaded to ESP32.
* **Webots Role**: Executes path using serial input, sensor feedback, and closed-loop control.
* **ESP32 Role**: Computes optimal path and sends start/stop signal.
* **Connection**: Serial (USB) using `/dev/ttyUSB0` or COM port.

## Path Planning Algorithm: Dijkstra

### Overview

Dijkstra's algorithm finds the shortest path in a graph with non-negative weights. It tracks minimum cumulative distances from the start node to all others, using a priority queue to visit the most promising node next.

### Pseudocode

```python
function dijkstra(graph, start):
    dist = {node: inf for node in graph}
    dist[start] = 0
    prev = {}
    queue = PriorityQueue()
    queue.put((0, start))

    while not queue.empty():
        cost, current = queue.get()
        for neighbor, weight in graph[current]:
            alt = cost + weight
            if alt < dist[neighbor]:
                dist[neighbor] = alt
                prev[neighbor] = current
                queue.put((alt, neighbor))

    return dist, prev
```

### Implementation Notes

* Graph is predefined in ESP32 as an adjacency list.
* Uses tuple `(node, cost)` format.
* Sends computed path as list of node names (e.g., `['A', 'B', 'E', 'F']`).

## Path Execution and Re-Planning

* **Initial Path**: Sent once at simulation start.
* **Re-Planning**: Not automatic in current version (planned in future update).
* **User Control**: ESP32 has push-button to send new path on-demand.

## Future Work

* Add real-time obstacle detection in Webots.
* Enable dynamic re-planning on ESP32.
* Implement A\* support on ESP32.
* Add GUI for path visualization.

---
