FROM osrf/ros:noetic-desktop-full

##################
# parameters
##################

# Set IP of Franka control PC
ENV FCI_IP=172.16.0.2

##################
# libfranka build
##################

# Download and build the required franka libraries
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    libpoco-dev \
    libeigen3-dev \
    wget \
    software-properties-common

# Install an updated version of cmake
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
RUN apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
RUN apt-get update && apt-get install -y cmake

RUN git clone --recursive https://github.com/frankaemika/libfranka --branch 0.10.0 # only for FR3
WORKDIR /libfranka
RUN mkdir build
WORKDIR /libfranka/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
RUN cmake --build . && cmake --install .

# Make a Debian package and install it
RUN cpack -G DEB
RUN dpkg -i libfranka*.deb

##################
# franka_ros build
##################

# Setup ROS catkin workspace
WORKDIR /catkin_ws
RUN mkdir src
SHELL ["/bin/bash", "-c"]  # so we can use `source`
RUN source /opt/ros/noetic/setup.sh && catkin_init_workspace src

# Download and build
RUN git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
RUN rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
RUN source /opt/ros/noetic/setup.sh && catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/libfranka/build

# Add lines to the bashrc file that source ROS
RUN echo "source /ros_entrypoint.sh" >> /root/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc
