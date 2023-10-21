ARG ROS_DISTRO="humble"
FROM ros:$ROS_DISTRO as base
ARG BRANCH="humble"

ENV ROS_UNDERLAY /ros2_control_ws
WORKDIR $ROS_UNDERLAY/src

# Use Cyclone DDS as middleware
# Install foxglove-bridge for remote visualization
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-foxglove-bridge

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

## add packages
ADD https://raw.githubusercontent.com/ros-controls/ros2_control/$BRANCH/ros2_control.$ROS_DISTRO.repos ros2_control.repos
RUN vcs import < ros2_control.repos
COPY src/rrbot_cam ./rrbot_cam
COPY src/rrbot_cam_msgs ./rrbot_cam_msgs
COPY src/rrbot_cam_platform_controller ./rrbot_cam_platform_controller

## install dependencies
RUN apt-get update \
    && rosdep update \
    && rosdep install -iy --from-paths . \
    && rm -rf /var/lib/apt/lists/

## compile
RUN cd $ROS_UNDERLAY \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install

## source entrypoint setup
RUN sed --in-place --expression \
      '$isource "$ROS_UNDERLAY/install/setup.bash"' \
      /ros_entrypoint.sh

#####################
# Development Image #
#####################
# From base as dev

# # Dev container arguments
# ARG USERNAME=devuser
# ARG UID=1000
# ARG GID=${UID}

# # Install extra tools for development
# RUN apt-get update \
#     && apt-get install -y --no-install-recommends \
#         tmux vim \
#     && rm -rf /var/lib/apt/lists/

# # Create new user and home directory
# RUN groupadd --gid $GID $USERNAME \
#     && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
#     && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
#     && chmod 0440 /etc/sudoers.d/${USERNAME} \
#     && mkdir -p /home/${USERNAME} \
#     && chown -R ${UID}:${GID} /home/${USERNAME}

# # Set the user and source entrypoint in the user's .bashrc file
# USER ${USERNAME}
# RUN echo "source /entrypoint.sh" >> /home/${USERNAME}/.bashrc