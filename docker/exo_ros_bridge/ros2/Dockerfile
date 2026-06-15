FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    --break-system-packages \
    --no-cache-dir \
    pyzmq

WORKDIR /ros2_ws

COPY dynamic_biped /ros2_ws/src/dynamic_biped
COPY bridge/ros2_publisher.py /bridge/ros2_publisher.py

RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install

CMD source /opt/ros/jazzy/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    exec python3 /bridge/ros2_publisher.py
