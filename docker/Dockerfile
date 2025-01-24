FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
apt-get install -y python3.10 python3-pip libosmesa6-dev libgl1-mesa-glx libglfw3 libglew-dev libpng-dev && \
rm -rf /var/lib/apt/lists/*


FROM base AS mujoco_python

RUN mkdir /models

ARG MUJOCO_VERSION

RUN python3 -m pip install mujoco==${MUJOCO_VERSION}

WORKDIR /home

ENTRYPOINT [ "/usr/bin/bash" ]


FROM base AS environment

RUN mkdir /models

ARG PYTHON_REQUIREMENTS_PATH

COPY --link ${PYTHON_REQUIREMENTS_PATH}/requirements.txt .

RUN python3 -m pip install -r requirements.txt

WORKDIR /home

ENTRYPOINT [ "/usr/bin/bash" ]
