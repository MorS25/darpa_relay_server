# Base Image: Ubuntu Bionic w/ ROS core
FROM ros:melodic-ros-core

# NVIDIA Container Runtime for OpenGL support.
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# For GUI support, add a new user with the same UID/GID as the host.
# This should be set as a build parameter, but defaults to 1000.
ARG UID=1000
ENV USERNAME subt
RUN useradd -U --uid ${UID} -ms /bin/bash $USERNAME \
 && echo "$USERNAME:$USERNAME" | chpasswd

# Update package lists
RUN apt-get update

# Install rviz
RUN apt-get install ros-melodic-rviz -q -y

# Install Eigen3
RUN apt-get install libeigen3-dev

# Set the user
USER $USERNAME

# Add the mapping server files to the image
WORKDIR /home/$USERNAME
RUN mkdir subt_ws subt_ws/src subt_ws/src/subt_mapping_server
ADD . subt_ws/src/subt_mapping_server

# Build the package
WORKDIR /home/$USERNAME/subt_ws
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make install"

# Expose necessary ports
EXPOSE 8000

# Run the mapping server.
ENTRYPOINT ["src/subt_mapping_server/docker/docker_entrypoint.bash"]
