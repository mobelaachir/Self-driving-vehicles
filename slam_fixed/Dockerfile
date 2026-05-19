ARG REPO_NAME="duckiebot_slam_dtproject"
ARG DESCRIPTION="Duckiebot SLAM project based on template-ros"
ARG MAINTAINER="Mohamed"
ARG ARCH=arm64v8

FROM duckietown/dt-ros-commons:ente-${ARCH}

# Install Python dependencies
RUN pip3 install opencv-python matplotlib numpy

# Copy launchers only — packages are mounted at runtime by dts devel run
COPY ./launchers /launchers
RUN chmod +x /launchers/*.sh

WORKDIR /code/catkin_ws

ENTRYPOINT ["/launchers/default.sh"]
