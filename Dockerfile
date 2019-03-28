# docker build --tag=hashcode2016 .
FROM ubuntu:16.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common

RUN add-apt-repository ppa:jonathonf/python-3.6
RUN apt-get update

RUN apt-get install -y --no-install-recommends \
    build-essential \
    python3.6 \
    python3.6-dev \
    python3-pip \
    python3.6-venv \
    && apt-get clean \
    && rm -rf /var/lib/apt-get/lists/* /tmp/* /var/tmp/*


# Install Swi-prolog
RUN add-apt-repository ppa:swi-prolog/stable
RUN apt-get update && apt-get install -y --no-install-recommends \
  swi-prolog


# update pip
RUN python3.6 -m pip install pip --upgrade
RUN python3.6 -m pip install wheel


# Install Jinja2
RUN apt-get install -y python-jinja2


RUN alias python=python3
