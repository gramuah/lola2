FROM ros:noetic-ros-core-focal
SHELL ["/bin/bash", "-c"]

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
      build-essential \
      python3-rosdep \
      python3-rosinstall \
      python3-vcstools && \
    rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && rosdep update

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends ros-noetic-ros-base=1.5.0-1* &&\
    rm -rf /var/lib/apt/lists/*

# Install additional ROS Packages
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-noetic-joy \
        ros-noetic-ros-control \
        ros-noetic-ros-controllers \
        ros-noetic-realsense2-camera

# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip  \
        python-is-python3

# Install additional Python modules
RUN pip3 install pyserial

WORKDIR /catkin_lola2
COPY ./catkin_lola2/ ./
RUN source /opt/ros/noetic/setup.bash && catkin_make

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]