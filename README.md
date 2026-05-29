# 2025-tfg-diego-lopez

## Installation

### Requirements

Before installing the project, make sure the following dependencies are installed:

- ROS 2 Jazzy (tested on Ubuntu 24.04)
- Gazebo Harmonic
- ONNX Runtime

#### ONNX Runtime

The ONNX Runtime library is required to run inference in the `g1_locomotion` package.

You can check whether it is already installed by running:

```bash
ldconfig -p | grep onnxruntime
```

If no output appears, install ONNX Runtime with the following commands:
```bash
wget https://github.com/microsoft/onnxruntime/releases/download/v1.17.1/onnxruntime-linux-x64-1.17.1.tgz

tar -xvzf onnxruntime-linux-x64-1.17.1.tgz
sudo cp -r onnxruntime-linux-x64-1.17.1/include/* /usr/local/include/
sudo cp onnxruntime-linux-x64-1.17.1/lib/libonnxruntime.so* /usr/local/lib/
sudo ldconfig
```
After installation, run the verification command again to confirm that ONNX Runtime was installed successfully.

### Workspace Configuration

First, create a ROS 2 workspace:
```bash
mkdir -p ws/src 
cd ws/src
```

Clone this repository inside the `src` directory.

Then, go back to the workspace root and build the packages:
```bash
cd ..

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```
The repository is now ready to use.

## Usage

Once the repository has been installed and the workspace has been sourced, start the locomotion inference node with:

```bash
ros2 run g1_locomotion g1_locomotion_main
```

This command starts the neural network inference used to control the robot.

After that, launch the simulation:
```bash
ros2 launch g1_sim launch_g1_sim.launch.py
```
This command starts the Gazebo simulation environment.

**!IMPORTANT**

Make sure to start the inference node **before** launching the simulation.

Otherwise, the robot may fall before the controller begins sending commands.
