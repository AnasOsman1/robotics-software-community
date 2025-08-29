# this is the image of ROS2 that is pulled (downloaded an included in the local image)
FROM osrf/ros:humble-desktop-full

# we are using bash
SHELL ["/bin/bash", "-c"]

# here we specify the work directory inside of our container, everything will be placed inside
WORKDIR /temp_subscriber

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV MYUSER=user
ENV MYGROUP=users
ENV MYUID=1000
ENV MYGID=1000

RUN apt update && \
    apt install -y sudo && \ 
    addgroup --gid ${MYGID} ${MYUSER} && \
    adduser --gecos '' --disabled-password --uid ${MYUID} --gid ${MYGID} ${MYUSER} && \
    adduser ${MYUSER} sudo && \
    adduser ${MYUSER} adm && \
    echo "${MYUSER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/nopasswd

USER ${MYUSER}

RUN sudo apt update && \ 
    sudo apt install -y \ 
    iputils-ping \ 
    software-properties-common \
    ros-humble-rmw-cyclonedds-cpp \ 
    ros-dev-tools \
    ros-humble-rosbridge-server
# here, we are copying "src" folder from our workspace folder to the newly created folder 
# cpp_ws/src inside of our container. That is, we are creating "cpp_ws" folder 
# int the container and another subfolder inside of that folder called "src"
# the path in the container is /temp_subscriber/cpp_ws/src
COPY src cpp_ws/src

# here we are starting from /temp_subscriber and moving to /temp_subscriber/cpp_ws and we are sourcing the 
# base environment that is now inside of our container and we are building the package
USER root
RUN cd cpp_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build

# here, we are copying another file from our local computer to the root folder "/" (forward slash)
# inside of our container 
# see entrypoint.sh file for more details    
COPY src/cpp_ws/entrypoint.sh /entrypoint.sh

# here we are specifying the entry point file
ENTRYPOINT ["/entrypoint.sh"]
# after we run the package the command "bash" will be executed, and 
# we will enter a local bash terminal inside of our container
CMD ["bash"]