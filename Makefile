# 定義變數
DOCKER_IMAGE=ros-noetic-custom
DOCKER_CONTAINER=ros_noetic_container
DOCKERFILE=Dockerfile
SKETCH_NAME=my_sketch

# 預設目標：build 和 run
all: build run

# 建立 Docker 映像檔
build:
		docker build -t $(DOCKER_IMAGE) -f $(DOCKERFILE) .

# Activate the container

# If in rasberry pi 4, we need to load usb of web camera and arduino mega
## Add --device=/dev/ttyUSB0:/dev/ttyUSB0 to let docker access the arduino
## Add --device=/dev/video0:/dev/video0 to let docker access the web camera 

# Use amcl to localize the robot, so we need to load the rplidar
run:
		xhost +local:root
		docker run -it --rm \
			--name $(DOCKER_CONTAINER) \
			--privileged \
			--ulimit nofile=1024:524288 \
			-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
			-v /home/dunkun/Documents/control_lab/HW06/ros_docker_project/catkin_ws:/catkin_ws \
			-v $(PWD)/arduino:/arduino \
			-e DISPLAY=$(DISPLAY) \
			--device /dev/dri \
			$(DOCKER_IMAGE)


# 清理已建立的容器和映像檔
clean:
		docker system prune -f
		docker rmi $(DOCKER_IMAGE)

# 安裝 Gazebo 和相關工具
install_gazebo:
		docker run -it --rm \ 
		-v /home/dunkun/Documents/control_lab/HW06/ros_docker_project/catkin_ws:/catkin_ws \
		$(DOCKER_IMAGE) bash -c "apt-get install -y gazebo11 libgazebo11-dev"

attach:
		@sudo docker exec -it $(DOCKER_CONTAINER) /bin/zsh -c "source /catkin_ws/devel/setup.zsh && /bin/zsh"