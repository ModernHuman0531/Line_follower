# ROS Noetic Docker Development Environment On PC to test turtlebot Line-follower

This repository contains a Dockerfile and Makefile to set up a complete ROS Noetic development environment with ZSH, neovim, Gazebo, and other useful tools.

## Objective
This docker file is mainly the testing version on my PC for the final competition in Control Lab from NYCU before actually implementing on raspi4 in turtlebot3.
Noted that since it just the test version on PC, it doesn't load any device yet, we just use simple video to test the effect. 

## Features

- ROS Noetic with full desktop installation
- ZSH shell with Oh-My-ZSH
- Powerlevel10k theme for ZSH
- ZSH plugins: autosuggestions, syntax-highlighting
- Neovim for file editing
- Gazebo simulation environment with GUI support
- Persistent configuration with externally mounted config files
- Volume-mounted catkin workspace for easy development
- Opencv modules to identify the lanes
- Install rosserial to help the communication between ros's docker package and arduino
- Navigation package install

## Prerequisites

- Docker installed on your system
- X server for GUI applications (if on Linux, this is already available)


## Setup Instructions

1. Clone this repository:
```bash
git clone <repository-url>
cd <repository-directory>
```

2. First-time setup:
```bash
make build
```

This will:
- Create necessary directories
- Create config files if they don't exist
- Build the Docker image
- Start a new container

3. For subsequent runs, you can use:
```bash
make run
```

4. To clean up (remove containers and images):
```bash
make clean
```
5. Go to same docker when already have one opened
```bash
make attach
```
## Directory Structure

- `catkin_ws/`: Your ROS workspace (mounted inside the container)
  - `./src/lane_follower` : Node for detect the lane use opencv and publish the /offset and /motor_pwm message
    - `./scripts/lane_detector_node.py` : Use opencv's find contour to identify the left and right lane, calculate and publish the offset between mid lane and car.
    - `./scripts/control_node.py` : Simple p-control node to convert offset to the pwm of the chassis, to help us easier do PID control in arduino
    - `./asset` : Put the testing video here for checking the lane_detection part.
- `config/`: Contains ZSH and Powerlevel10k configuration files
  - `.zshrc`: ZSH configuration
  - `.p10k.zsh`: Powerlevel10k theme configuration
- `arduino/` : Your chassis control program
  - PID.ino : Receive /motor_pwm message to use PID method to control the chassis
  - Fuzzy.ino : Use fuzzy control to control the chassis, subscribe the /offset message to do the control

## Using the Environment

- The catkin workspace is mounted at `/home/ros/catkin_ws` inside the container
- Any changes you make in the workspace are preserved between sessions
- To build your ROS packages, use `catkin_make` inside the workspace
- To run Gazebo, use `gazebo`, `gzclient`, or `gzserver` commands

## Configuration

You can customize the setup by modifying:

- `Dockerfile`: To add more packages or change the setup
- `Makefile`: To change container names, volume mounts, or other Docker run options
- `config/.zshrc`: To customize your ZSH shell
- `config/.p10k.zsh`: To customize your Powerlevel10k theme

## Troubleshooting

### GUI applications don't work

Make sure your X server is running and properly configured:

- On Linux: Make sure you've run `xhost +local:docker` before starting the container

### Gazebo crashes or displays rendering errors

This might be due to graphics driver issues. Try running with different OpenGL-related environment variables:
```bash
LIBGL_ALWAYS_SOFTWARE=1 gazebo
```

