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
                libgtest-dev                   \
                libbenchmark-dev               \
                libopencv-dev                  \
                libgl1-mesa-dev                \
                libglew-dev

RUN cd /usr/src/googletest && cmake -DBUILD_GTEST=ON -DBUILD_GMOCK=ON . \
    && make -j && make install

RUN mkdir code
WORKDIR code
RUN git config --global http.sslVerify false
RUN git clone https://github.com/stevenlovegrove/Pangolin.git
RUN cd Pangolin && mkdir build && cd build && cmake .. -DBUILD_TESTS=OFF -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF && make && make install
RUN rm -rf Pangolin
RUN git clone https://github.com/yse/easy_profiler.git
RUN cd easy_profiler && mkdir build && cd build && cmake -DEASY_PROFILER_NO_GUI=ON .. && make && make install
RUN rm -rf easy_profiler 
RUN git clone https://github.com/fmtlib/fmt.git
RUN cd fmt && mkdir build && cd build && cmake .. && make && make install
RUN rm -rf fmt
RUN git clone https://github.com/wow2006/dso.git dso
RUN cd dso
RUN mkdir build
WORKDIR build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_VERBOSE_MAKEFILE=ON ../dso
CMD make
