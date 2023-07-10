FROM moveit/moveit2:humble-release

RUN \
    # Update apt package list as previous containers clear the cache
    apt update && \
    apt-get -qq dist-upgrade &&\ 
    #apt-get install python3-pip -y &&\
    apt-get install ros-dev-tools -y &&\
    apt-get install ros-humble-rqt* -y &&\
    apt-get install stress -y &&\
    apt-get install python3-vcstool -y


COPY ./src/ /ws/src/

WORKDIR /ws/src

RUN git clone https://github.com/Hannes-Beckmann/servo_bug_moveit_resources.git -b humble 

WORKDIR /ws

RUN rosdep install -y --from-paths src --ignore-src --rosdistro humble --as-root=apt:false
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc
RUN echo 'source /ws/install/setup.bash' >> /root/.bashrc

RUN . /opt/ros/humble/setup.sh &&\
    colcon build 

#docker run --rm -it -e DISPLAY=host.docker.internal:0.0 --mount type=bind,source=E:\Dokumente\voraus\voraus-hannes-studienarbeit\colcon_ws,target=/yu_node  --privileged voraushannesstudienarbeit
#docker run --rm -it --mount type=bind,source=/home/voraus/voraus-hannes-studienarbeit/colcon_ws,target=/yu_node  --privileged --network host --name roscontainer voraushannesstudienarbeit
#docker exec -it roscontainer /bin/bash
