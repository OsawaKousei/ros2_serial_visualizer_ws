FROM n622jwith/ros2_basic_base:latest

# gui tools
RUN apt-get update && \
    apt-get install -y libqt5gui5 && \
    rm -rf /var/lib/apt/lists/*
ENV QT_DEBUG_PLUGINS=1

# install rqt
RUN apt-get update &&\
    apt-get install -y ros-humble-rqt-* &&\
    rm -rf /var/lib/apt/lists/*

# Add user and group, then allow sudo
ARG UID
ARG GID
ARG PASSWORD
ARG USER_NAME
ARG GROUP_NAME
ARG GIT_USER_NAME
ARG GIT_USER_EMAIL

RUN groupadd -g ${GID} ${GROUP_NAME}
RUN useradd -u ${UID} -g ${GID} -G sudo -s /bin/bash -m ${USER_NAME}
RUN echo "${USER_NAME}:${PASSWORD}" | chpasswd
RUN echo "Defaults visiblepw" >> /etc/sudoers
RUN echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${USER_NAME}

# git config
RUN git config --global user.name "${GIT_USER_NAME}" && \
    git config --global user.email "${GIT_USER_EMAIL}"

# setup ros2
COPY setup.sh /home/${USER_NAME}/setup.sh
RUN bash ~/setup.sh

CMD ["/bin/bash"]
