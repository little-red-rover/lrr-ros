FROM ros:noetic
SHELL ["/bin/bash", "-c"]

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /ros_setup.bash

### tooling
RUN apt-get update && \
    apt-get install -y --no-install-recommends python3 python3-pip wget curl

RUN pip3 install protobuf cryptography pathlib

### gazebo setup
# RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
#     apt-get update && \
#     apt-get install -y --no-install-recommends ros-$ROS_DISTRO-ros-gz

# By default gazebo calculates lidar data using the GPU, but GPU operations aren't possible cross platform in Docker.
# Setting this ENV variable causes it to be calculated on the CPU.
# https://github.com/gazebosim/gz-sensors/issues/26
RUN echo "export LIBGL_ALWAYS_SOFTWARE=true" >> /ros_setup.bash

### Setup ROS workspace
RUN mkdir -p /little_red_rover_ws/src/little_red_rover
COPY ./little_red_rover /little_red_rover_ws/src/little_red_rover

WORKDIR /little_red_rover_ws

RUN apt-get update && \
    rosdep update && \ 
    rosdep install --from-paths src --ignore-src -y 

RUN source /ros_setup.bash && catkin_make
RUN echo "source /little_red_rover_ws/devel/setup.bash" >> /ros_setup.bash

### Dev env setup
RUN apt-get update && \
    apt-get install -y --no-install-recommends black iputils-ping python3-venv unzip && \
    pip3 install black

RUN PROTOC_ZIP=protoc-27.3-linux-x86_64.zip && \
    curl -OL https://github.com/protocolbuffers/protobuf/releases/download/v27.3/$PROTOC_ZIP && \
    unzip -o $PROTOC_ZIP -d /usr/local bin/protoc && \
    unzip -o $PROTOC_ZIP -d /usr/local 'include/*' && \
    rm -f $PROTOC_ZIP


RUN echo "alias lrr_install='(cd /little_red_rover_ws && apt update && rosdep update && rosdep install --from-paths src --ignore-src -y)'" >> /root/.bashrc
RUN echo "alias lrr_build='(cd /little_red_rover_ws && catkin_make)'" >> /root/.bashrc
# RUN echo "alias lrr_run='ros2 launch little_red_rover lrr.launch.py'" >> /root/.bashrc
RUN echo "alias lrr_connect='. /tools/wifi_auth.bash'" >> /root/.bashrc

RUN echo "export IDF_PATH=/tools/idf" >> /root/.bashrc

RUN echo "source /ros_setup.bash" >> /root/.bashrc

CMD bash -c "roscore"
