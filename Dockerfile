From ros:foxy

RUN apt-get update
RUN apt-get install --yes --force-yes vim git python3-pip wget unzip screen
RUN apt-get install --yes --force-yes libgeos-dev liblapack-dev libeigen3-dev gfortran python3-h5py

RUN python3 -m pip install cython scipy numpy pandas sympy nose
RUN python3 -m pip install numpy --upgrade
RUN python3 -m pip install shapely geographiclib pymap3d astropy pysolar spherical-geometry
RUN python3 -m pip install fastkml pyserial pyaml 

RUN mkdir /opt/gradle 
COPY tmp/gradle-6.7-bin.zip /opt/gradle
RUN cd /opt/gradle && unzip gradle-6.7-bin.zip 
ENV PATH="/opt/gradle/gradle-6.7/bin:${PATH}"

RUN apt-get install --yes --force-yes openjdk-8-jdk

RUN git clone --recursive https://github.com/eProsima/Fast-RTPS.git -b 1.8.x /opt/Fast-RTPS-1.8.2
RUN mkdir /opt/Fast-RTPS-1.8.2/build
RUN cd /opt/Fast-RTPS-1.8.2/build && cmake -DTHIRDPARTY=ON -DSECURITY=ON .. 
RUN cd /opt/Fast-RTPS-1.8.2/build && make
RUN cd /opt/Fast-RTPS-1.8.2/build && make install

RUN git clone --recursive https://github.com/eProsima/Fast-RTPS-Gen.git -b v1.0.4 /opt/Fast-RTPS-Gen
RUN cd /opt/Fast-RTPS-Gen && gradle assemble && gradle install

RUN python3 -m pip install pyros-genmsg 
COPY robots/pybots /opt/pybots
RUN cd /opt/pybots && python3 -m pip install .
CMD cd /opt/autonomy && bash
