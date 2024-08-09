FROM continuumio/miniconda3:master AS cc_base
# See https://github.com/CloudCompare/CloudComPy/blob/master/doc/BuildLinuxCondaDocker.md

RUN . /opt/conda/etc/profile.d/conda.sh && \
    conda activate && \
    conda install mamba -n base -c conda-forge && \
    mamba create --name CloudComPy310 python=3.10 && \
    conda activate CloudComPy310 && \
    mamba install -c conda-forge "boost=1.74" "cgal=5.4" cmake draco ffmpeg "gdal=3.5" jupyterlab laszip "matplotlib=3.5" "mysql=8.0" "numpy=1.22" "opencv=4.5" "openmp=8.0" "pcl=1.12" "pdal=2.4" "psutil=5.9" pybind11 "qhull=2020.2" "qt=5.15.4" "scipy=1.8" sphinx_rtd_theme spyder tbb tbb-devel "xerces-c=3.2"

RUN apt-get update && apt-get install -y gfortran g++ make libgl1 libgl-dev

RUN . /opt/conda/etc/profile.d/conda.sh && \
    conda activate CloudComPy310 && \
    cd && rm -rf CloudComPy && git clone --recurse-submodules https://gitlab.com/openfields1/CloudComPy.git && \
    cd CloudComPy && git checkout CloudComPy_docker-20230216 --recurse-submodules

ARG FBXINC=noplugin
ARG FBXLIB=noplugin
ARG CORKINC=noplugin
ARG CORKLIB=noplugin
ARG LIBIGL=noplugin
ARG OPENCASCADE=noplugin

COPY docker/$FBXINC /root/fbxsdk/include/
COPY docker/$FBXLIB /root/fbxsdk/lib/
COPY docker/$CORKINC /root/cork/src/
COPY docker/$CORKLIB /root/cork/lib/
COPY docker/$LIBIGL /root/libigl/
COPY docker/$OPENCASCADE /root/occt/
COPY docker/genCloudComPy_Conda310_docker.sh /root/

RUN cd /root && \
    if [ -f fbxsdk/include/fbxsdk.h ]; then \
        sed -i 's/QFBX:BOOL="0"/QFBX:BOOL="1"/g' genCloudComPy_Conda310_docker.sh; \
    fi; \
    if [ -f cork/src/cork.h ]; then \
        sed -i 's/QCORK:BOOL="0"/QCORK:BOOL="1"/g' genCloudComPy_Conda310_docker.sh; \
    fi; \
    cd /root && chmod +x genCloudComPy_Conda310_docker.sh && ./genCloudComPy_Conda310_docker.sh

RUN echo "#!/bin/bash\n\
. /opt/conda/etc/profile.d/conda.sh\n\
cd /opt/installConda/CloudComPy310\n\
. bin/condaCloud.sh activate CloudComPy310\n\
export QT_QPA_PLATFORM=offscreen\n\
cd /opt/installConda/CloudComPy310/doc/PythonAPI_test\n\
ctest" > /execTests.sh && chmod +x /execTests.sh

FROM ros:humble-ros-core AS main

COPY --from=cc_base /opt/installConda/CloudComPy310 /CloudComPy 

RUN apt-get update -y && apt-get install -y \
        wget libgl-dev xvfb x11-apps python3-colcon-common-extensions\
    && rm -rf /var/lib/apt/lists/* 

RUN wget -O Miniforge3.sh "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh" && \
    bash Miniforge3.sh -b -p "/conda" && \
    . "/conda/etc/profile.d/conda.sh" && \
    . "/conda/etc/profile.d/mamba.sh" && \
    mamba create --name CloudComPy310 python=3.10 && \
    mamba activate CloudComPy310 && \
    mamba install -y "boost=1.74" "cgal=5.4" draco ffmpeg "gdal=3.5" laszip "matplotlib=3.5" "mysql=8.0" "numpy=1.22" "opencv=4.5" "openmp=8.0" "pcl=1.12" "pdal=2.4" "psutil=5.9" pybind11 quaternion "qhull=2020.2" "qt=5.15.4" "scipy=1.8" sphinx_rtd_theme tbb tbb-devel "xerces-c=3.2"

ENV DISPLAY=:0

COPY docker/setup_cloudcompy310.sh /
COPY docker/setup_cloudcompy310.sh /
COPY docker/ros_entrypoint.sh /
COPY ros2_livox_cloudcompare /ros2_ws/src/ros2_livox_cloudcompare
COPY pointcloud_registration_msgs /ros2_ws/src/pointcloud_registration_msgs

RUN apt-get update -y && apt-get install -y \
        build-essential \
    && rm -rf /var/lib/apt/lists/* 

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd ros2_ws \
    && colcon build --symlink-install 

COPY docker/run.sh /

CMD ["bash", "-c", "su ros -c /run.sh"]
