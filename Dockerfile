# 20.04 is not the latest lts but necessary for ROS2 foxy
FROM ubuntu:20.04

# Update, install python essentials, and git
RUN apt-get update && apt-get install -y python3 python3.8-venv &&  apt-get install -y git

# ROS2 foxy Dockerfile build tutorial: https://devanshdhrafani.github.io/blog/2021/04/15/dockerros2.html
# Set-up ROS2 package sources
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 foxy packages
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y ros-foxy-desktop

# Set-up ROS2 workspace and clone tutorials
WORKDIR /root/dev_ws/src
RUN git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
WORKDIR /root/dev_ws

# Install rosdep (resolves missing dependencies) and colcon (next iter of catkin_make)
RUN apt-get install python3-rosdep -y && rosdep init && rosdep update
RUN rosdep install -i --from-path src --rosdistro foxy -y
RUN apt-get install python3-colcon-common-extensions -y

# Setup the python venv
RUN python3 -m venv /opt/venv
# todo:: this may have to change with the workdir set above
ENV PATH="/opt/venv/bin:$PATH"

# Copy the pip requirements
COPY requirements.txt .

# Install python requirements
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir --upgrade -r requirements.txt

# Install Open3D system dependencies and opencv
RUN apt-get update && apt-get install --no-install-recommends -y \
    libgl1 \
    libgomp1 \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Copy everything to the docker container
COPY . ./

# Command that is run when docker run is called.
ENTRYPOINT ["python3", "main.py"]

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
