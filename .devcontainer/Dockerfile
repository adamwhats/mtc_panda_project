FROM althack/ros2:humble-dev 

# Add the moveit nightly packages
RUN echo "deb [trusted=yes] https://raw.githubusercontent.com/moveit/moveit2_packages/jammy-${ROS_DISTRO}/ ./" | sudo tee /etc/apt/sources.list.d/moveit_moveit2_packages.list
RUN echo "yaml https://raw.githubusercontent.com/moveit/moveit2_packages/jammy-${ROS_DISTRO}/local.yaml ${ROS_DISTRO}" | sudo tee /etc/ros/rosdep/sources.list.d/1-moveit_moveit2_packages.list

# Install and build libfranka
RUN apt-get update \
   && apt-get install -y libpoco-dev libeigen3-dev \
   && git clone https://github.com/frankaemika/libfranka.git --recursive -b 0.11.0\
   && cd libfranka  \
   && mkdir build && cd build \
   && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF  .. \
   && cmake --build . -j$(nproc) \
   && cpack -G DEB \
   && sudo dpkg -i libfranka-*.deb
   
# Install required ROS packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends ros-${ROS_DISTRO}-rviz2 \
   ros-${ROS_DISTRO}-control-toolbox \
   ros-${ROS_DISTRO}-cv-bridge \
   ros-${ROS_DISTRO}-moveit-py \
   ros-${ROS_DISTRO}-moveit-configs-utils \
   ros-${ROS_DISTRO}-moveit-planners \
   ros-${ROS_DISTRO}-moveit-setup-assistant \
   ros-${ROS_DISTRO}-moveit-simple-controller-manager \
   ros-${ROS_DISTRO}-moveit-resources-panda-moveit-config \
   ros-${ROS_DISTRO}-ros2-control \
   ros-${ROS_DISTRO}-joint-state-broadcaster \
   ros-${ROS_DISTRO}-joint-state-publisher \
   ros-${ROS_DISTRO}-joint-trajectory-controller \
   #
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   # && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

# Install required python modules
RUN pip install opencv-python pyassimp scipy transitions setuptools==58.2.0

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
