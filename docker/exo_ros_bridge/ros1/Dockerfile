FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pyzmq

WORKDIR /ros1_ws

COPY dynamic_biped /ros1_ws/src/dynamic_biped
COPY bridge/ros1_receiver.py /bridge/ros1_receiver.py

RUN source /opt/ros/noetic/setup.bash && \
    catkin_make

CMD source /opt/ros/noetic/setup.bash && \
    source /ros1_ws/devel/setup.bash && \
    exec python3 /bridge/ros1_receiver.py
