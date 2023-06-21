ARG ROS_DISTRO:=foxy
ARG BASE_IMAGE=osrf/ros:foxy-desktop

FROM osrf/ros:foxy-desktop
RUN . /opt/ros/foxy/setup.sh

# Set the necessary environment variables
ENV QT_X11_NO_MITSHM 1
ENV LIBGL_ALWAYS_INDIRECT 1
ENV ROS_DOMAIN_ID 6
ENV DEBIAN_FRONTEND noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"] 

# Install dependencies for GPU support
RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-utils \
    x11-apps \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    intel-gpu-tools \
    libgles2-mesa-dev \
    clinfo \
    ocl-icd-opencl-dev

# # Check for NVIDIA GPU
# RUN apt-get update && apt-get install -y \
#     pciutils
# RUN if lspci | grep -i nvidia; then \
#         # NVIDIA GPU found, install NVIDIA drivers and libraries
#         echo "Installing NIVIDIA GPU Drivers"; \
#         apt-get update && apt-get install -y \
#             nvidia-driver-470 \
#             libnvidia-gl-470 \
#             libgles2-mesa-dev; \
#         echo "export NVIDIA_VISIBLE_DEVICES=all" >> /etc/profile; \
#         echo "export NVIDIA_DRIVER_CAPABILITIES=all" >> /etc/profile; \
#         # ENV NVIDIA_VISIBLE_DEVICES all; \
#         # ENV NVIDIA_DRIVER_CAPABILITIES all; \
#     else \
#         # No NVIDIA GPU found, install Intel drivers and libraries
#         echo "Installing Intel GPU Drivers"; \
#         apt-get update && apt-get install -y \
#             intel-gpu-tools \
#             mesa-utils \
#             libgles2-mesa-dev; \
#         # ENV DOCKER_BUILDKIT=1; \
#         # ENV DOCKER_DEFAULT_RUNTIME=intel; \
#         echo "export DOCKER_BUILDKIT=1" >> /etc/profile; \
#         echo "export DOCKER_DEFAULT_RUNTIME=intel" >> /etc/profile; \
#     fi
ENV QT_X11_NO_MITSHM 1
ENV LIBGL_ALWAYS_INDIRECT 1
ENV CL_CONTEXT_EMULATOR_DEVICE_INTEL 1
ENV DOCKER_BUILDKIT 1
ENV DOCKER_DEFAULT_RUNTIME intel

# Set up X11 forwarding
ENV DISPLAY :0

# Update, install python essentials
RUN apt-get update && apt-get install -y python3-pip python3-tk

# Install sensor_msgs_py from common_interfaces
RUN apt-get install -y ros-rolling-sensor-msgs-py

# Install UR robot driver and its dependencies
RUN apt-get install -y ros-${ROS_DISTRO}-ur-robot-driver

# Install Open3D system dependencies and opencv
RUN apt-get update && apt-get install --no-install-recommends -y \
    libgl1 \
    libgomp1 \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /home/ros_ws/src

# Copy the pip requirements
COPY requirements.txt .

# Install requirements
RUN pip install --upgrade --no-cache-dir pip \
    pip install --upgrade --no-cache-dir -r requirements.txt

# Copy everything to the docker container
COPY main.py ./
COPY app/ ./app
COPY data/ ./data

# Command that is run when docker run is called.
CMD ["python3", "main.py"]

# i am stuck here:
# docker run -it -d flip/master:latest
# docker ps
# docker exec -it <NAMES> bash
# - should be in ~/dev_ws#
# cd ../..
# - should be in ~#
# source opt/ros/foxy/setup.bash
# echo "source /opt/ros/foxy/setup.bash" >> ~/ .bashrc (didnt do anything)
# - now ros2 command works but cant got back to ~/dev_ws
# printenv | grep -i ROS
# ros2 
# ros2 run demo_nodes_cpp talker
# ros2 run demo_nodes_cpp listener
# - but when cloning tutorial and ros2 launch <launch file> or ros2 run talker or ./talker or python talker.py lib (rospy or rclpy) is still missing
# # 20.04 is not the latest lts but necessary for ROS2 foxy
# FROM ubuntu:20.04
# # ROS2 foxy Dockerfile build tutorial: https://devanshdhrafani.github.io/blog/2021/04/15/dockerros2.html
# # Set-up ROS2 package sources
# RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# # Install ROS2 foxy packages
# RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y ros-foxy-desktop

# # Set-up ROS2 workspace and clone tutorials
# WORKDIR /root/dev_ws/src
# RUN git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
# WORKDIR /root/dev_ws

# # Install rosdep (resolves missing dependencies) and colcon (next iter of catkin_make)
# RUN apt-get install python3-rosdep -y && rosdep init && rosdep update
# RUN rosdep install -i --from-path src --rosdistro foxy -y
# RUN apt-get install python3-colcon-common-extensions -y

# # Setup the python venv
# RUN python3 -m venv /opt/venv
# # todo:: this may have to change with the workdir set above
# ENV PATH="/opt/venv/bin:$PATH"

# ------------------------------------------------------------------------------------------------
# ENV DEBIAN_FRONTEND=noninteractive
# ENV SHELL /bin/bash
# ENV ROS_DOMAIN_ID 6
# ENV NVIDIA_DRIVER_CAPABILITIES all
# SHELL ["/bin/bash", "-c"] 

# # Install necessary dependencies for GPU support
# RUN apt-get update && apt-get install -y \
#     clinfo \
#     ocl-icd-opencl-dev

# # Set the default runtime for Docker to "intel"
# ENV DOCKER_BUILDKIT=1
# ENV DOCKER_DEFAULT_RUNTIME=intel

# # Check for NVIDIA GPU and set the runtime accordingly
# ARG NVIDIA_GPU=false
# ENV DOCKER_BUILDKIT=1
# ENV DOCKER_DEFAULT_RUNTIME=intel
# RUN if [ "$NVIDIA_GPU" = "true" ]; then \
#         apt-get update && apt-get install -y --no-install-recommends \
#             nvidia-container-runtime; \
#         export DOCKER_DEFAULT_RUNTIME=nvidia; \
#     fi

# RUN curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | apt-key add -

# RUN distribution=$(. /etc/os-release;echo ${ID}${VERSION_ID}) \
#     && curl -s -L https://nvidia.github.io/nvidia-docker/${distribution}/nvidia-docker.list | tee /etc/apt/sources.list.d/nvidia-docker.list

# # Install GPU drivers and NVIDIA Container Toolkit
# RUN apt-get update && apt-get install -y \
#     curl \
#     gnupg \ 
#     nvidia-container-toolkit \
#     nvidia-docker2