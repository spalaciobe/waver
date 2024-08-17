<h1 align="center">waver_rviz package</h1>

## Overview

The `waver_rviz` package provides visualization for the Wave Rover robot within the RViz environment. This package includes the necessary configuration files and a launch file to visualize the robot's sensors, state, and environment in real-time.

## Visualization

### Prerequisites
Before using the `waver_rviz` package, ensure you have the following installed:
- Docker. [See for details](https://github.com/GGomezMorales/waver?tab=readme-ov-file#how-to-use).
- [waver_description package](https://github.com/GGomezMorales/waver/tree/noetic/waver_description).

### Visualization steps

1. Clone the repository:
   ```bash
   git clone https://github.com/GGomezMorales/waver.git
   cd waver
   ```

2. Build the Docker image:
   ```bash
   ./scripts/build.sh
   ```

3. Run the Docker container:
   There are three ways to run a container, depending on your host machine:
      - **`run_docker.sh`:** Uses the standard Docker command to run the container.
         ```bash
         ./scripts/run_docker.sh
         ```
      - **`run_cpu.sh`:** Uses [rocker](https://github.com/osrf/rocker) to run the container on a CPU environment.
         ```bash
         ./scripts/run_cpu.sh
         ```
      - **`run_nvidia.sh`:** Uses [rocker](https://github.com/osrf/rocker) to run the container with NVIDIA GPU support.
         ```bash
         ./scripts/run_nvidia.sh
         ```

   After running the container:
   ```bash
   root@10bb71e2357e:/waver_ws# 
   ```

4. ROS environment setup:
   The Docker image includes an alias to simplify the process of sourcing and building packages within a catkin workspace. To set up the ROS environment, use the following command:
   ```bash
   sros
   ```

5. RViz launch:
   Finally, to launch the RViz visualization with the Wave Rover, use the following command:
   ```bash
   roslaunch waver_viz rviz.launch
   ```
