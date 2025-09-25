FROM ubuntu:24.04
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
RUN sudo apt install -y git nano python-is-python3 wget curl

# install pixi and uv
USER $USERNAME
WORKDIR /home/$USERNAME
RUN sudo apt install -y curl
RUN curl -fsSL https://pixi.sh/install.sh | sh
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

# add dotfiles to force pixi shell activation
RUN git clone https://github.com/gandres42/dotfiles.git && mv dotfiles .dotfiles
RUN echo 'source ~/.dotfiles/setup.bash' >> ~/.bashrc
ENV SHELL /bin/bash
ENV PIXI_ACTIVATE true
CMD ["/bin/bash"]