FROM ros:jazzy-ros-base AS ros2

# Copy requirement files and install dependencies
COPY docker/requirements.txt .
RUN apt-get update && apt-get install --no-install-recommends -y $(cat requirements.txt)
RUN rm requirements.txt

# Defines a workspace folder.
WORKDIR /home/ws
