FROM ubuntu:22.04
ARG USERNAME=USERNAME
ARG CONTAINER_ID=CONTAINER_ID
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ENV DEBIAN_FRONTEND noninteractive

# create non-root user and add to sudo
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# update and install packages
RUN apt-get update && apt-get upgrade -y
RUN apt-get install software-properties-common -y
RUN add-apt-repository ppa:ros-for-jammy/noetic
RUN apt-get install -y python3-pip ros-noetic-desktop -y

# install python packages
RUN pip install open3d ompl catkin_tools

# dev environment setup
USER $USERNAME
WORKDIR /home/$USERNAME
RUN sudo apt install -y git nano python-is-python3 wget curl
RUN echo '[pip]\nprefix = ".local"' > uv.toml
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
RUN git clone https://github.com/gandres42/dotfiles.git && mv dotfiles .dotfiles
RUN echo 'source ~/.dotfiles/setup.bash' >> ~/.bashrc
ENV SHELL /bin/bash
ENV CONTAINER_ID $CONTAINER_ID
CMD ["/bin/bash"]