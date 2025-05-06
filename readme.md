# ROS Noetic Docker Development Environment

This repository contains a Dockerfile and Makefile to set up a complete ROS Noetic development environment with ZSH, neovim, Gazebo, and other useful tools.

## Features

- ROS Noetic with full desktop installation
- ZSH shell with Oh-My-ZSH
- Powerlevel10k theme for ZSH
- ZSH plugins: autosuggestions, syntax-highlighting
- Neovim for file editing
- Gazebo simulation environment with GUI support
- Persistent configuration with externally mounted config files
- Volume-mounted catkin workspace for easy development

## Prerequisites

- Docker installed on your system
- X server for GUI applications (if on Linux, this is already available)
- On Windows, you might need VcXsrv or similar X server
- On macOS, you might need XQuartz

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

## Directory Structure

- `catkin_ws/`: Your ROS workspace (mounted inside the container)
- `config/`: Contains ZSH and Powerlevel10k configuration files
  - `.zshrc`: ZSH configuration
  - `.p10k.zsh`: Powerlevel10k theme configuration

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
- On Windows/macOS: Ensure your X server is running and properly configured

### Permission issues with mounted volumes

The Dockerfile creates a user with UID and GID matching your host system user. If you're still experiencing permission issues, check that the UID and GID in the Makefile match your user's.

### Gazebo crashes or displays rendering errors

This might be due to graphics driver issues. Try running with different OpenGL-related environment variables:
```bash
LIBGL_ALWAYS_SOFTWARE=1 gazebo
```

## License

This project is open-source and available under the MIT License.
