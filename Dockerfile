FROM ubuntu:20.04
ENV DEBIAN_FRONTEND noninteractive

# -----Choose ROS distributions-----
ENV ROS_DISTRO noetic


# -----locale config-----
RUN echo "-----Setting up locale-----"
RUN apt-get update && \
    apt-get install -y locales && \
    dpkg-reconfigure locales && \
    locale-gen ja_JP ja_JP.UTF-8 && \
    update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8
ENV LC_ALL ja_JP.UTF-8
ENV LANG ja_JP.UTF-8
ENV LANGUAGE ja_JP.UTF-8


# -----apt source list config-----
RUN echo "-----Install apt sources-----"
RUN apt-get update && \
    apt-get install -y \
        curl \
        gnupg2 \
        lsb-release \
        git \
        vim \
        wget \
        unzip \
        net-tools


# -----Install Python-----
RUN echo "-----Install Python-----"
RUN apt-get install python3-pip -y && \
    apt-get install libgl1-mesa-dev -y && \
    pip3 install opencv-python && \
    pip3 install mysql-connector-python 


# -----Install ROS1-----
RUN echo "-----Install ROS-----"
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-get update && apt-get install -y ros-$ROS_DISTRO-desktop 
    

# -----Install ROS plugins-----
RUN echo "-----Install ROS Plugins-----"
RUN apt-get update && \
    apt-get install -y  \
        python3-catkin-tools \
        python3-rosdep \
        python3-argcomplete  \
	ros-${ROS_DISTRO}-usb-cam \
        ros-${ROS_DISTRO}-rosbridge-server \
        ros-${ROS_DISTRO}-hector-trajectory-server


# -----Personal ROS settings-----
RUN echo "-----Personal ROS settings-----"
RUN mkdir -p /root/Programs/Settings
COPY settings/ros/.ros_config /.ros_config
COPY settings/ros/rosSetup.txt  /root/Programs/Settings/rosSetup_CRLF.txt
RUN sed "s/\r//g" /root/Programs/Settings/rosSetup_CRLF.txt > /root/Programs/Settings/rosSetup.txt && \
    cat /root/Programs/Settings/rosSetup.txt >> /root/.bashrc && \
    echo "" >> /root/.bashrc


# -----Install powerline-shell-----
RUN echo "-----Install powerline-shell-----"
RUN pip3 install powerline-shell
COPY settings/powerline/powerlineSetup.txt /root/Programs/Settings/powerlineSetup_CRLF.txt
RUN sed "s/\r//g" /root/Programs/Settings/powerlineSetup_CRLF.txt > /root/Programs/Settings/powerlineSetup.txt && \
    cat /root/Programs/Settings/powerlineSetup.txt >> /root/.bashrc && \
    echo "" >> /root/.bashrc
RUN mkdir -p /root/.config/powerline-shell
COPY settings/powerline/config.json /root/.config/powerline-shell/config.json


# -----Install x-server-----
RUN echo "-----Install x-server-----"
RUN apt-get update && \
    apt-get install -y xserver-xorg x11-apps


# -----ORB_SLAM3 Configuration-----
RUN echo "-----Install ORB_SLAM3-----"
RUN echo "Step1: Pangolin"
RUN mkdir /root/ORB_SLAM
RUN cd /root/ORB_SLAM && \
    git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
RUN cd /root/ORB_SLAM/Pangolin && \
    yes | ./scripts/install_prerequisites.sh recommended && \
    cmake -B build -GNinja && \
    cmake --build build
RUN echo "Step2: ORB_SLAM3"
RUN cd /root/ORB_SLAM && \
    git clone -b focal https://github.com/koukemo/ORB_SLAM3.git
RUN cd /root/ORB_SLAM/ORB_SLAM3 && \
    chmod +x build.sh && \
    ./build.sh


# -----Install orb_slam3_ros_wrapper-----
RUN echo "-----Install orb_slam3_ros_wrapper-----"
RUN mkdir -p /root/ros1_ws/src && \
    cd /root/ros1_ws/src && \
    git clone -b focal https://github.com/koukemo/orb_slam3_ros_wrapper.git
RUN cd /root/ros1_ws && \
    /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; catkin build"
RUN cp /root/ORB_SLAM/ORB_SLAM3/Vocabulary/ORBvoc.txt /root/ros1_ws/src/orb_slam3_ros_wrapper/config/


# -----Starting directory-----
RUN echo "-----Setup is complete!-----"
WORKDIR /root/ros1_ws


# -----Open port-----
RUN echo "-----Open port 9090 for ROS#-----"
EXPOSE 9090
