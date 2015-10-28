FROM python:2.7

# LTLMoP non-python dependencies (these can't be installed with just pip)
RUN apt-get update && apt-get install -y \
  openjdk-7-jdk \
  python-numpy \
  python-scipy && \
  apt-get autoremove -y

# link system wide dependencies (numpy + scipy)
ENV PYTHONPATH=$PYTHONPATH:/usr/lib/python2.7/dist-packages/

# install python dependencies (do this system wide to match numpy/scipy)
WORKDIR /LTLMoP
COPY ./requirements.txt ./requirements.txt
RUN pip install -r requirements.txt

# add source code to container
ADD ./src /LTLMoP/src

# install java dependencies, taken from build.sh (can't run cd w/docker)
WORKDIR /LTLMoP/src/etc/jtlv/GROne
RUN javac -sourcepath . -cp ../jtlv-prompt1.4.0.jar *.java
