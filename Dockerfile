FROM ubuntu:14.04
MAINTAINER agilgur5

# LTLMoP non-python dependencies (these can't be installed with just pip)
RUN apt-get update && apt-get install -y --no-install-recommends \
  openjdk-7-jdk \
  python-numpy \
  python-scipy \
  python-pip && \
  apt-get autoremove -y

# link system wide dependencies (numpy + scipy)
ENV PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages/

# install python dependencies
WORKDIR /LTLMoP
COPY ./requirements.txt ./requirements.txt
# python-dev necessary for Polygon2 (--no-install-recommends also breaks it)
RUN apt-get install -y python-dev && apt-get autoremove -y && \
  pip install -r requirements.txt

# add source code to container
ADD ./src /LTLMoP/src

# install java dependencies, taken from build.sh (can't run cd w/docker)
WORKDIR /LTLMoP/src/etc/jtlv/GROne
RUN javac -sourcepath . -cp ../jtlv-prompt1.4.0.jar *.java
