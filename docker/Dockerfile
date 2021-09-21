# Base image
FROM ros:melodic 

ENV TZ=Europe/Amsterdam
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Basic tools
RUN apt-get update && \
    apt-get install vim nano git tmux wget curl python-pip net-tools iputils-ping  -y

# Install additional ros packages
RUN apt-get update && apt-get install ros-melodic-rosbridge-server ros-melodic-joy -y
RUN pip install adafruit-pca9685

# Install packages for web application
RUN curl -sL https://deb.nodesource.com/setup_12.x | bash -
RUN apt-get update && \
    apt-get install nodejs -y
RUN npm install http-server -g

#Invalid key problem solved!
RUN curl http://repo.ros2.org/repos.key | sudo apt-key add -
# Install packages for camera use
RUN apt-get update
RUN apt-get install ros-melodic-web-video-server ros-melodic-usb-cam -y

# Update
RUN sudo apt-get update
RUN sudo apt-get upgrade -y

# # Trying OpenCV last time
# RUN apt-get install libhdf5-dev -y
# RUN apt-get install -y libhdf5-serial-dev
# RUN apt-get install -y libatlas-base-dev
# RUN apt-get install jasper -y
# RUN apt-get install python3-pip -y
# RUN pip3 install --upgrade pip setuptools wheel
# RUN apt-get install python3-dev
# RUN apt-get install libgl1-mesa-glx
# RUN pip3 install scikit-build
# RUN pip3 install numpy
# RUN apt-get install -y libqtgui4
# RUN apt-get install -y libqt4-test
# RUN pip3 install scikit-build
# RUN pip3 install urllib3
# RUN pip3 install opencv-contrib-python

# #try2
# RUN sudo apt install software-properties-common -y
# #libjasper pip
# RUN git clone https://github.com/jasperproject/jasper-client.git jasper && \
#     chmod +x jasper/jasper.py && \
#     pip install --upgrade setuptools
# RUN pip install -r jasper/client/requirements.txt
# RUN apt-get install -y build-essential \
#     cmake \
#     wget \
#     git \
#     unzip \
#     yasm \
#     pkg-config \
#     libjpeg-dev \
#     libtiff-dev \
#     libpng-dev \
#     libavcodec-dev \
#     libavformat-dev \
#     libswscale-dev \
#     libv4l-dev \
#     libatlas-base-dev \
#     gfortran \
#     libtbb2 \
#     libtbb-dev \
#     libpq-dev \
#     && apt-get -y clean all \
#     && rm -rf /var/lib/apt/lists/*
# 
# RUN pip install numpy
# 
# RUN add-apt-repository universe
# RUN apt-get update
# 
# RUN pip install feedparser
# RUN apt-get install -y python3-pip
# RUN pip3 install --upgrade pip setuptools wheel
# 
# # Install OpenCV
# RUN pip3 install opencv-contrib-python==4.4.0.46


# Add ros sourcing to bashrc
RUN echo ". /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Create Ros workspace
ENV EXOMY_WS=/root/exomy_ws
RUN mkdir -p $EXOMY_WS/src

WORKDIR /root

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
