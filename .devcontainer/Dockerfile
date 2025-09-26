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

# install miniconda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh \
    && bash /tmp/miniconda.sh -b -p /opt/miniconda \
    && rm /tmp/miniconda.sh \
    && /opt/miniconda/bin/conda clean -afy
ENV PATH="/opt/miniconda/bin:$PATH"

# create conda environment with robostack-jazzy channel and install ROS noetic
WORKDIR /home/$USERNAME
USER $USERNAME
RUN conda tos accept
RUN conda config --add channels conda-forge \
    && conda config --add channels robostack-noetic \
    && conda create -n noetic ros-noetic-desktop pip -y \
    && conda clean -afy
RUN conda init
SHELL ["conda", "run", "-n", "noetic", "/bin/bash", "-c"]
RUN pip install ompl open3d scipy
RUN echo "conda activate noetic" >> /home/$USERNAME/.bashrc
ENV SHELL /bin/bash