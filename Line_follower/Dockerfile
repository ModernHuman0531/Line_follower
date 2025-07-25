# Use ROS Noetic official image for desktop environment
# FROM ros:noetic

# If use in raspberry pi 4, use this line
FROM arm64v8/ros:noetic


# Environment settings
ENV DEBIAN_FRONTEND=noninteractive


# Install dependencies
# Since rasberry pi can't run gazebo, we don't need to install gazebo
# Remove /gazebo11 and libgazebo11-dev
RUN apt-get update && apt-get install -y \
  zsh \
  curl \
  git \
  wget \
  vim \
  lsb-release \
  build-essential \
  libboost-all-dev \
  python3-pip \
  libglu1-mesa \
  libxi6 \
  libxmu6 \
  libxrender1 \
  libxt6 \
  libx11-dev \
  libc6-dev \
  python3-rosdep \
  python3-catkin-tools \
  v4l-utils \
  && rm -rf /var/lib/apt/lists/*

# 安裝 oh-my-zsh, powerlevel10k, zsh-autosuggestions, zsh-syntax-highlighting
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
RUN git clone --depth 1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/themes/powerlevel10k
RUN git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
RUN git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting

# 設定 Zsh
RUN echo "ZSH_THEME=\"powerlevel10k/powerlevel10k\"" >> ~/.zshrc
RUN echo "plugins=(git zsh-autosuggestions zsh-syntax-highlighting)" >> ~/.zshrc

# 安裝 nvim
RUN apt-get update && apt-get install -y neovim

# Install python package for the for line detection
# 安裝 numpy 和 matplotlib，確保版本相容
# 強制升級 numpy 到 1.20 以上版本，然後再安裝 matplotlib
# RUN pip install --no-cache-dir "numpy>=1.20"
RUN apt-get update && apt-get install -y python3-numpy

# 安裝 OpenCV in local computer
# RUN pip install --no-cache-dir opencv-python
# If install opencv in raspberry pi 4, use this line
RUN apt-get update && apt-get install -y python3-opencv

# Install ROS OpenCV(cv bridge) to use OpenCV in ROS
# Install rosserial and rosserial-arduino to enable communication between ROS and Arduino
RUN apt-get update && apt-get install -y \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-camera-info-manager \
  ros-noetic-sensor-msgs \
  ros-noetic-vision-opencv \
  libxcb-xinerama0 \
  ros-noetic-rosserial \
  ros-noetic-rosserial-arduino \
  ros-noetic-usb-cam \
  libxcb-xinerama0 \
  qt5-qmake qtbase5-dev-tools qtbase5-dev \
  && rm -rf /var/lib/apt/lists/*

# Install arduino-cli
# Don't need this line if in raspberry pi 4, since already install arduino IDE

# RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
# ENV PATH="/root/bin:${PATH}"

# Initialize arduino-cli and update the index and install the core
# Don't need this line if in raspberry pi 4, since already install arduino IDE
#RUN arduino-cli config init && \
 # arduino-cli core update-index && \
  # arduino-cli core install arduino:avr

# For rasberry pi 4, install GTK module to avoid canberra-gtk-module error
RUN apt-get update && apt-get install -y \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libgtk2.0-0 \
    libgtk-3-0 \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Modify the setting of rt.cmake to enable the rasbeer pi 4 run catkin_make
COPY docker_fix/rt.cmake /opt/ros/noetic/share/catkin/cmake/tools/rt.cmake

# 複製本機的 Zsh 設定檔
COPY ./zsh-config/.zshrc /root/.zshrc

# 確保 Gazebo 可以顯示 GUI
# Don't need this line if in raspberry pi 4
# RUN apt-get install -y libx11-dev libglu1-mesa

# 設置容器啟動時進入 Zsh
CMD ["/usr/bin/zsh"]
