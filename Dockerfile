# Copyright (c) 2025 Hiroyuki Okada
# All rights reserved.
FROM ubuntu:22.04 AS cpu
LABEL maintainer="Hiroyuki Okada <hiroyuki.okada@okadanet.org>"
LABEL org.okadanet.vendor="Hiroyuki Okada" \
      org.okadanet.dept="TIDBots" \
      org.okadanet.version="1.0.0" \
      org.okadanet.released="November 1, 2025"

SHELL ["/bin/bash", "-c"]

RUN sed -i.org -e 's|archive.ubuntu.com|ubuntutym.u-toyama.ac.jp|g' /etc/apt/sources.list
RUN apt-get clean
RUN /bin/echo -e "Acquire::http::Timeout \"300\";\n\
   Acquire::ftp::Timeout \"300\";" >> /etc/apt/apt.conf.d/timeout_setting

# Install packages without prompting the user to answer any questions
ENV DEBIAN_FRONTEND noninteractive 
RUN apt-get update && apt-get install -y \
locales language-pack-ja-base language-pack-ja locales fonts-takao \
sudo lsb-release ca-certificates wget curl git subversion libssl-dev \
nano vim htop \
dbus-x11 mesa-utils x11-utils x11-apps \
terminator xterm \
build-essential software-properties-common gdb valgrind && \
apt-get clean && rm -rf /var/lib/apt/lists/*

# Install tmux 3.2
RUN apt-get update && apt-get install -y automake autoconf pkg-config libevent-dev libncurses5-dev bison && \
apt-get clean && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/tmux/tmux.git && \
cd tmux && git checkout tags/3.2 && ls -la && sh autogen.sh && ./configure && make -j8 && make install

# Set locale
RUN  locale-gen ja_JP ja_JP.UTF-8  \
  && update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8 \
  && add-apt-repository universe
# Locale
ENV LANG ja_JP.UTF-8
ENV TZ=Asia/Tokyo

RUN apt-get update && apt-get install -y \
  build-essential cmake g++ \
  iproute2 gnupg gnupg1 gnupg2 \
  libcanberra-gtk* \
  python3-pip  python3-tk \
  git wget curl \
  x11-utils x11-apps terminator xterm xauth mesa-utils\
  terminator xterm nano vim htop \
  software-properties-common gdb valgrind sudo


# Install ROS2
RUN apt update \
  && apt install -y --no-install-recommends \
     curl gnupg2 lsb-release python3-pip vim wget build-essential ca-certificates
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update \
#  && apt upgrade \
  && DEBIAN_FRONTEND=noninteractive \
  && apt install -y --no-install-recommends \
     ros-humble-desktop   python3-colcon-common-extensions python3-rosdep \
  && rm -rf /var/lib/apt/lists/*

# Add user and group
ARG UID
ARG GID
ARG USER_NAME
ARG GROUP_NAME
ARG PASSWORD
RUN groupadd -g $GID $GROUP_NAME && \
    useradd -m -s /bin/bash -u $UID -g $GID -G sudo $USER_NAME && \
    echo $USER_NAME:$PASSWORD | chpasswd && \
    echo "$USER_NAME   ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER ${USER_NAME}

RUN source /opt/ros/humble/setup.bash  \
    && sudo apt-get update  \
    && sudo rosdep init \
    && rosdep update

RUN mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src && \
      git clone -b humble https://github.com/hsr-project/hsrb_controllers.git && \
      git clone -b humble https://github.com/hsr-project/hsrb_common.git && \
      git clone -b humble https://github.com/hsr-project/hsrb_drivers.git && \
      git clone -b humble https://github.com/hsr-project/hsrb_interfaces.git  && \
      git clone -b humble https://github.com/hsr-project/hsrb_launch.git && \
      git clone -b humble https://github.com/hsr-project/hsrb_manipulation.git && \
      git clone -b humble https://github.com/hsr-project/hsrb_moveit.git && \
      git clone -b humble https://github.com/hsr-project/hsrb_rosnav.git && \
      git clone -b humble https://github.com/hsr-project/hsrb_simulator.git && \
      git clone -b humble https://github.com/hsr-project/hsr_common.git && \
      git clone -b humble https://github.com/hsr-project/hsrb_teleop.git && \
      git clone -b humble https://github.com/hsr-project/tmc_gazebo.git && \
      git clone -b humble https://github.com/hsr-project/tmc_teleop.git && \
      git clone -b humble https://github.com/hsr-project/tmc_common.git && \
      git clone -b humble https://github.com/hsr-project/tmc_common_msgs.git && \
      git clone -b humble https://github.com/hsr-project/tmc_drivers.git && \
      git clone -b humble https://github.com/hsr-project/tmc_database.git && \
      git clone -b humble https://github.com/hsr-project/tmc_manipulation.git && \
      git clone -b humble https://github.com/hsr-project/tmc_manipulation_base.git && \
      git clone -b humble https://github.com/hsr-project/tmc_manipulation_planner.git && \
      git clone -b humble https://github.com/hsr-project/tmc_realtime_control.git && \
      git clone -b humble https://github.com/hsr-project/tmc_voice.git && \
      git clone -b humble https://github.com/hsr-project/tmc_navigation.git && \
      rm -rf hsrb_launch/hsrb_robot_launch && \
      rm -rf hsrb_simulator/hsrb_rviz_simulator && \
      rm -rf tmc_drivers/tmc_pgr_camera

RUN cd ~/ros2_ws && \
      source /opt/ros/humble/setup.bash && \
      rosdep install --from-paths . -y --ignore-src && \
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release


# Config (if you wish)
RUN mkdir -p ~/.config/terminator/
COPY assets/terminator_config /home/$USER_NAME/.config/terminator/config 
COPY assets/tmux.conf /home/$USER_NAME/.tmux.conf
COPY assets/tmux.session.conf /home/$USER_NAME/.tmux.session.conf 

# .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USER_NAME/.bashrc
RUN echo "source /home/${USER_NAME}/ros2_ws/install/setup.bash" >> /home/$USER_NAME/.bashrc



# entrypoint
COPY ./assets/entrypoint.sh /
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

WORKDIR /home/$USER_NAME
CMD ["/bin/bash"]


#FROM nvidia/cuda:12.9.1-cudnn-devel-ubuntu20.04 AS gpu
#FROM nvidia/cuda:13.0.1-cudnn-devel-ubuntu22.04 AS gpu
#FROM nvidia/cudagl:11.3.0-devel-ubuntu20.04 AS gpu
FROM nvidia/opengl:1.0-glvnd-devel-ubuntu22.04 AS gpu

SHELL ["/bin/bash", "-c"]
ARG DEBIAN_FRONTEND=noninteractive

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics




# Install packages without prompting the user to answer any questions
ENV DEBIAN_FRONTEND noninteractive 
RUN apt-get update && apt-get install -y \
locales language-pack-ja-base language-pack-ja locales fonts-takao \
sudo lsb-release ca-certificates wget curl git subversion libssl-dev \
nano vim htop \
dbus-x11 mesa-utils x11-utils x11-apps \
terminator xterm \
build-essential software-properties-common gdb valgrind && \
apt-get clean && rm -rf /var/lib/apt/lists/*

# Install tmux 3.2
RUN apt-get update && apt-get install -y automake autoconf pkg-config libevent-dev libncurses5-dev bison && \
apt-get clean && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/tmux/tmux.git && \
cd tmux && git checkout tags/3.2 && ls -la && sh autogen.sh && ./configure && make -j8 && make install

# Set locale
RUN  locale-gen ja_JP ja_JP.UTF-8  \
  && update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8 \
  && add-apt-repository universe
# Locale
ENV LANG ja_JP.UTF-8
ENV TZ=Asia/Tokyo

RUN apt-get update && apt-get install -y \
  build-essential cmake g++ \
  iproute2 gnupg gnupg1 gnupg2 \
  libcanberra-gtk* \
  python3-pip  python3-tk \
  git wget curl \
  x11-utils x11-apps terminator xterm xauth mesa-utils\
  terminator xterm nano vim htop \
  software-properties-common gdb valgrind sudo

# Install ROS2
RUN apt update \
  && apt install -y --no-install-recommends \
     curl gnupg2 lsb-release python3-pip vim wget build-essential ca-certificates
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update \
#  && apt upgrade \
  && DEBIAN_FRONTEND=noninteractive \
  && apt install -y --no-install-recommends \
     ros-humble-desktop   python3-colcon-common-extensions python3-rosdep python3-nose \
  && rm -rf /var/lib/apt/lists/*

RUN sudo rosdep init && rosdep update
RUN source /opt/ros/humble/setup.bash && \
    mkdir -p /ros2_ws/src && cd /ros2_ws/src && \
    git clone -b humble https://github.com/hsr-project/hsrb_controllers.git && \
      git clone -b humble https://github.com/hsr-project/hsrb_common.git && \
      git clone -b humble https://github.com/hsr-project/hsrb_drivers.git && \
      git clone -b humble https://github.com/hsr-project/hsrb_interfaces.git  && \
      git clone -b humble https://github.com/hsr-project/hsrb_launch.git && \
      git clone -b humble https://github.com/hsr-project/hsrb_manipulation.git && \
      git clone -b humble https://github.com/hsr-project/hsrb_moveit.git && \
      git clone -b humble https://github.com/hsr-project/hsrb_rosnav.git && \
      git clone -b humble https://github.com/hsr-project/hsrb_simulator.git && \
      git clone -b humble https://github.com/hsr-project/hsr_common.git && \
      git clone -b humble https://github.com/hsr-project/hsrb_teleop.git && \
      git clone -b humble https://github.com/hsr-project/tmc_gazebo.git && \
      git clone -b humble https://github.com/hsr-project/tmc_teleop.git && \
      git clone -b humble https://github.com/hsr-project/tmc_common.git && \
      git clone -b humble https://github.com/hsr-project/tmc_common_msgs.git && \
      git clone -b humble https://github.com/hsr-project/tmc_drivers.git && \
      git clone -b humble https://github.com/hsr-project/tmc_database.git && \
      git clone -b humble https://github.com/hsr-project/tmc_manipulation.git && \
      git clone -b humble https://github.com/hsr-project/tmc_manipulation_base.git && \
      git clone -b humble https://github.com/hsr-project/tmc_manipulation_planner.git && \
      git clone -b humble https://github.com/hsr-project/tmc_realtime_control.git && \
      git clone -b humble https://github.com/hsr-project/tmc_voice.git && \
      git clone -b humble https://github.com/hsr-project/tmc_navigation.git && \
      rm -rf hsrb_launch/hsrb_robot_launch && \
      rm -rf hsrb_simulator/hsrb_rviz_simulator && \
      rm -rf tmc_drivers/tmc_pgr_camera
RUN source /opt/ros/humble/setup.bash && \
      cd /ros2_ws && \
      rosdep install --from-paths . -y --ignore-src && \
      colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 

  

# Add user and group
ARG UID
ARG GID
ARG USER_NAME
ARG GROUP_NAME
ARG PASSWORD
RUN groupadd -g $GID $GROUP_NAME && \
    useradd -m -s /bin/bash -u $UID -g $GID -G sudo $USER_NAME && \
    echo $USER_NAME:$PASSWORD | chpasswd && \
    echo "$USER_NAME   ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER ${USER_NAME}

#RUN source /opt/ros/humble/setup.bash \
#    && sudo apt-get update  \
#    && sudo rosdep init \
#    && rosdep update



# Config (if you wish)
RUN mkdir -p ~/.config/terminator/
COPY assets/terminator_config /home/$USER_NAME/.config/terminator/config 
COPY assets/tmux.conf /home/$USER_NAME/.tmux.conf
COPY assets/tmux.session.conf /home/$USER_NAME/.tmux.session.conf 

# .bashrc
# .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USER_NAME/.bashrc
RUN echo "source /home/${USER_NAME}/ros2_ws/install/setup.bash" >> /home/$USER_NAME/.bashrc


# entrypoint
COPY ./assets/entrypoint.sh /
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["/bin/bash"]
WORKDIR /home/$USER_NAME

