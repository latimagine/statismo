FROM ubuntu:bionic

RUN apt-get update && apt-get install -y apt-transport-https ca-certificates gnupg software-properties-common wget
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | apt-key add -

RUN apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main' && apt-get update

RUN apt-get install -y python3-pip python3-dev git cmake mesa-common-dev freeglut3-dev python3-pip swig\
  && cd /usr/local/bin \
  && ln -s /usr/bin/python3 python \
  && pip3 install --upgrade pip

RUN git clone https://github.com/kenavolic/statismo --branch master /usr/src/statismo

WORKDIR "/usr/src/statismo"

RUN mkdir build

WORKDIR "/usr/src/statismo/modules/VTK/wrapping"

RUN pip3 install -r requirements_tests.txt

WORKDIR "/usr/src/statismo/build"

RUN cmake ../superbuild -DBUILD_WRAPPING=ON -DBUILD_DOCUMENTATION=OFF -DBUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/src/statismo/dist

RUN make -j4

WORKDIR "/usr/src/statismo/build/Statismo-build"

RUN make install

ENV PYTHONPATH "${PYTHONPATH}:/usr/src/statismo/dist/lib/python3.6/site-packages:/usr/src/statismo/build/INSTALL/lib/python3.6/site-packages"
ENV LD_LIBRARY_PATH "${LD_LIBRARY_PATH}:/usr/src/statismo/dist/lib:/usr/src/statismo/build/INSTALL/lib/"
