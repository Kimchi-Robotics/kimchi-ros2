FROM ros:humble-ros-base AS ros2

# Arguments for building
ARG USERID
ARG USER

# Copy requirement files and install dependencies
COPY docker/requirements.txt .
RUN apt-get update && apt-get install --no-install-recommends -y $(cat requirements.txt)
RUN rm requirements.txt

# Creates the src folder of the workspace.
RUN mkdir -p /home/$USER/ws/src

# Updates
RUN sudo apt upgrade -y && sudo apt update && rosdep update

# Defines a workspace folder.
WORKDIR /home/$USER/ws
# Copy kimchi directory to workspace folder.
COPY kimchi/ ./src/kimchi

CMD ["/bin/bash"]
