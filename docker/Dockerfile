# Download base image
FROM osrf/ros:humble-desktop-full

# Arguments for building
ARG USERID
ARG USER

# Image name
ARG "Ubuntu base for ros2 humble development in ubuntu focal."

# Setup environment
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Copy installation scripts, and perform the dependency installation.
COPY docker/install.sh .
COPY docker/requirements.txt .
RUN ./install.sh
RUN rm install.sh requirements.txt

# Create a user with passwordless sudo
RUN adduser --uid $USERID --gecos "ekumen developer" --disabled-password $USER
RUN adduser $USER sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/$USER/.bashrc
USER $USER

# Creates the src folder of the workspace.
RUN mkdir -p /home/$USER/ws/src

# Adds to bashrc the ros humble overlay sourcing.
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc
# Adds to bashrc the gazebo related sourcing.
RUN echo "source /usr/share/gazebo/setup.bash" >> /home/$USER/.bashrc
# Adds colcon autocomplete
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USER/.bashrc

# Adds the rosdep update to the entrypoint.
RUN rosdep update

# Defines a workspace folder.
WORKDIR /home/$USER/ws

CMD ["/bin/bash"]
