# syntax=docker/dockerfile:1
# escape=\

# EXTERNAL REQUIREMENTS: https://github.com/jkk-research/mmdetection3d/ (dockerfile comment tag: #EXDEP_MMDET)
    # - clone the repository to the same directory as this file
# RUNTIME ENVIRONMENT VARIABLES ARE AT THE END OF FILE! (dockerfile comment tag: #RENV)

# cuda 11.8 as per install instructions
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04 AS fs_stack_x86-64

# general system setup (apt sources + install curl)
RUN apt update \
 && apt install software-properties-common -y \
 && add-apt-repository universe \
 && apt update \
 && apt install -y curl

# setting up ROS source for apt (linux [Debian] package manager)
RUN curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.1.0/ros2-apt-source_1.1.0.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
 && dpkg -i /tmp/ros2-apt-source.deb \
 && apt update && apt upgrade -y

# install ros base (some installers use interactive mode by default which stops the build so that must be disabled first)
RUN DEBIAN_FRONTEND=noninteractive apt install ros-humble-ros-base -y

# needed for ROS package dependencies
RUN apt install python3-rosdep -y \
 && apt install python3-colcon-common-extensions -y \
 && rosdep init || true && rosdep update

WORKDIR /ros2_ws

# some ROS2 node to be used (Kalman filter)
RUN apt update && apt install git -y && mkdir /ros2_ws/src && cd /ros2_ws/src \
 && git clone https://github.com/jkk-research/kalman_pos.git

# to avoid re-downloading, the formula_students_packages directory can be placed at the same level as this dockerfile (or the other way around)...
# ...and then just:
# COPY formula_student_packages /ros2_ws/src/formula_student_packages
# ...but perhaps it is easier to just re-download it:
RUN cd src && git clone https://github.com/szenergy/formula_student_packages.git

# pip package manager for python 3.x
RUN apt install -y python3-pip

# dependencies for lidar_detector (neural network) node
RUN pip install torch==2.1.1 torchvision==0.16.1 torchaudio==2.1.1 --index-url https://download.pytorch.org/whl/cu118

# dependencies for mmdetection3d [dependency]
RUN pip install mmengine \
 && pip install mmcv==2.1.0 -f https://download.openmmlab.com/mmcv/dist/cu118/torch2.1/index.html \
 && pip install mmdet==3.3.0

# [#EXDEP_MMDET]
# copy the cloned repo: https://github.com/jkk-research/mmdetection3d/
COPY mmdetection3d /ros2_ws/src/mmdetection3d

# todo?: script to directly download from git (requires authentication and therefore proper credentials handling or generating a scope-limited single-use token)

# install mmdetection3d and other lidar_detector (neural network) node dependencies
RUN cd /ros2_ws/src/mmdetection3d \
 && pip install -e . \
 && cd .. \
 && pip install cumm-cu118 \
 && pip install spconv-cu118 \
 && pip install transforms3d

# some other ROS2 node dependencies
RUN apt update && apt install ros-humble-tf-transformations -y && apt install ros-humble-pcl-ros -y && apt install libboost-all-dev -y

# numpy version for lidar_detector (neural network) node
RUN cd /ros2_ws/src \
 && git clone https://github.com/Box-Robotics/ros2_numpy.git \
 && cd ros2_numpy \
 && git switch humble \
 && cd ../.. \
 && . /opt/ros/humble/setup.sh \
 && colcon build --packages-select ros2_numpy 

# was needed, now seems to work without - try if relevant error massages appear
# RUN pip uninstall numpy scipy -y && pip install "numpy<2" "scipy<2.0"

# camera detection dependency
RUN pip install cv_bridge

# download weights (dep, binary)
RUN curl -L -o /ros2_ws/src/formula_student_packages/cone_detection_camera/detection/detection/resources/yolov7-tiny_last.weights \
    https://github.com/AlexeyAB/darknet/releases/download/yolov4/yolov7-tiny.weights

# MMDetection3D default path fix
# [PKG_FIX] MMDET3D  >>>>>
RUN sed -i 's:/home/dobayt/git/mmdetection3d:/ros2_ws/src/mmdetection3d:g' /ros2_ws/src/formula_student_packages/lidar_centerpoint_detector/lidar_centerpoint_detector/cone_detector.py \
 && sed -i 's:/home/dobayt/ros2_ws/src/formula_student_packages:/ros2_ws/src/formula_student_packages:g' /ros2_ws/src/formula_student_packages/lidar_centerpoint_detector/lidar_centerpoint_detector/cone_detector.py \
 && sed -i 's:/home/dobayt/git/mmdetection3d:/ros2_ws/src/mmdetection3d:g' /ros2_ws/src/formula_student_packages/lidar_centerpoint_detector/launch/launch_cone_detector.py \
 && sed -i 's:/home/dobayt/ros2_ws/src/formula_student_packages:/ros2_ws/src/formula_student_packages:g' /ros2_ws/src/formula_student_packages/lidar_centerpoint_detector/launch/launch_cone_detector.py
# <<<<< [PKG_FIX] MMDET3D

# planner dependency
RUN python3 -m pip install --upgrade pip setuptools && pip install "fsd-path-planning @ git+https://git@github.com/papalotis/ft-fsd-path-planning.git"

# building of ROS2 packages one by one (a general "colcon build" may not work well)
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select \
    lidar_centerpoint_detector \
    tf2_ros \
    kalman_pos  \
    deprojection_cpp \
    gmapping  \
    motion_tracking \
    lidar_pre_filter \
    cone_detection_lidar \
    detection \
    prcp_object_fusion \
    planner

# dependency for foxglove frontend
RUN apt update && apt install ros-humble-foxglove-bridge -y

ENTRYPOINT ["/bin/bash"]

# #RENV (runtime environment variables)
# detection mode (neural network = N [default], LiDAR-camera fusion = F)
ARG CONEDET_MODE=N
ENV CONEDET_MODE=${CONEDET_MODE}

# docker image size reduction
# RUN apt clean && rm -rf /var/lib/apt/lists/*