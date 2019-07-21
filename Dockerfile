FROM ubuntu:18.04
MAINTAINER Ahmed Abdel Aal <eng.ahmedhussein89@gmail.com>

 ENV TERM=xterm       \
     TZ=Europe/Berlin \
     DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y
RUN apt-get install -y --no-install-recommends \
                build-essential                \
                git                            \
                cmake                          \
                libsuitesparse-dev             \
                libeigen3-dev                  \
                libboost-all-dev               \
                libopencv-dev

RUN apt-get install -y --no-install-recommends \
                libgl1-mesa-dev                \
                libglew-dev

RUN mkdir code
WORKDIR code
RUN git config --global http.sslVerify false
RUN git clone https://github.com/stevenlovegrove/Pangolin.git
RUN cd Pangolin && mkdir build && cd build && cmake .. && make && make install
RUN git clone https://github.com/wow2006/dso.git dso
RUN cd dso && git checkout OpenCV4
RUN mkdir build
WORKDIR build
RUN cmake ../dso
CMD make
