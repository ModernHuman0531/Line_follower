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

# 啟動容器

run:
		docker run -it --rm \
			--name $(DOCKER_CONTAINER) \
			--privileged \
			--ulimit nofile=1024:524288 \
			-v /home/dunkun/Documents/control_lab/HW06/ros_docker_project/catkin_ws:/catkin_ws \
			-v $(PWD)/arduino:/arduino \
			-e DISPLAY=$DISPLAY \
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
		@sudo docker exec -it $(DOCKER_CONTAINER) /bin/zsh