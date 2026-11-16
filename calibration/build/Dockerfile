FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    ROS_DISTRO=noetic

RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates gnupg2 \
    && rm -rf /var/lib/apt/lists/* && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros1.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release ca-certificates git wget build-essential \
    python3 python3-pip python3-dev python3-catkin-tools python3-rosdep \
    python3-wstool python3-numpy python3-scipy python3-matplotlib \
    python3-yaml python3-opencv python3-pyqt5 \
    python3-wxgtk4.0 \
    libeigen3-dev libsuitesparse-dev libceres-dev \
    libtbb-dev libomp-dev libboost-all-dev \
    ros-noetic-ros-base ros-noetic-cv-bridge ros-noetic-image-transport \
    ros-noetic-camera-info-manager ros-noetic-message-filters \
    ros-noetic-sensor-msgs ros-noetic-rosbag \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true
RUN rosdep update --rosdistro noetic

WORKDIR /kalibr_workspace
RUN git clone --depth 1 https://github.com/ethz-asl/kalibr.git src/kalibr
COPY PlotCollection_headless.py /tmp/PlotCollection.py
RUN cp /tmp/PlotCollection.py /kalibr_workspace/src/kalibr/Schweizer-Messer/sm_python/python/sm/PlotCollection.py

RUN apt-get update && /bin/bash -c 'source /opt/ros/noetic/setup.bash && \
    rosdep install --from-paths src --ignore-src --rosdistro noetic -y && \
    catkin config --extend /opt/ros/noetic \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build --no-status --jobs 1 --parallel-packages 1' && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/noetic/setup.bash' >> /etc/bash.bashrc && \
    echo 'source /kalibr_workspace/devel/setup.bash' >> /etc/bash.bashrc

WORKDIR /data
CMD ["bash"]
