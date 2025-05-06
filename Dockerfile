# 使用 ROS Noetic 官方映像
FROM ros:noetic

# 環境設定
ENV DEBIAN_FRONTEND=noninteractive

# 安裝所需的套件
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
  gazebo11 \
  libgazebo11-dev \
  libglu1-mesa \
  libxi6 \
  libxmu6 \
  libxrender1 \
  libxt6 \
  libx11-dev \
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

# 設定 Gazebo
RUN apt-get update && apt-get install -y \
  libglu1-mesa \
  libxi6 \
  libxmu6 \
  libxrender1 \
  libxt6

# Install python package for the for line detection
# 安裝 numpy 和 matplotlib，確保版本相容
# 強制升級 numpy 到 1.20 以上版本，然後再安裝 matplotlib
RUN pip install --no-cache-dir numpy>=1.20
RUN pip install --no-cache-dir matplotlib

# 安裝 OpenCV
RUN pip install --no-cache-dir opencv-python

# Install ROS OpenCV(cv bridge) to use OpenCV in ROS
RUN apt-get update && apt-get install -y \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-camera-info-manager \
  ros-noetic-sensor-msgs \
  ros-noetic-vision-opencv \
  && rm -rf /var/lib/apt/lists/*

# Install arduino-cli
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
ENV PATH="/root/bin:${PATH}"

# Initialize arduino-cli and update the index and install the core
RUN arduino-cli config init && \
  arduino-cli core update-index && \
  arduino-cli core install arduino:avr


# 複製本機的 Zsh 設定檔
COPY ./zsh-config/.zshrc /root/.zshrc

# 確保 Gazebo 可以顯示 GUI
RUN apt-get install -y libx11-dev libglu1-mesa

# 確保使用者環境變數
ENV DISPLAY=${DISPLAY}

# 設置容器啟動時進入 Zsh
CMD ["/usr/bin/zsh"]
