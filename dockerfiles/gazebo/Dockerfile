FROM ubuntu:22.04

RUN apt-get update -y
RUN apt-get upgrade -y

RUN apt-get install lsb-release wget gnupg -y

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update -y
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get install gz-fortress -y