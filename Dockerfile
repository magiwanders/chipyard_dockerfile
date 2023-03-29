FROM ubuntu:focal

# Handle timezone problems
ENV TZ=Europe/Rome
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone 

# Install graphical terminal
RUN apt update && apt install xterm dbus-x11 -y

# Install build necessities
RUN apt-get install build-essential cmake curl tar git verilator sudo -y 

# Install Conda
RUN curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-$(uname)-$(uname -m).sh" && \
    (echo "\n"; echo "yes"; echo "/home/conda"; echo "yes") | bash Mambaforge-$(uname)-$(uname -m).sh
ENV PATH /home/conda/bin:$PATH

RUN sed -e '/[ -z "$PS1" ] && return/s/^/#/g' -i /root/.bashrc && \
    echo "source /home/conda/etc/profile.d/conda.sh" >> ~/.bashrc 
SHELL ["/bin/bash", "-l", "-c"]

RUN conda init bash && \
    echo "conda activate base" >> ~/.bashrc 

RUN conda install -n base conda-lock

# Install Chipyard
RUN cd home && \
    git clone https://github.com/ucb-bar/chipyard.git && \
    cd chipyard && \
    # git checkout 1.9.0 && \
    git checkout 1.8.1 && \
    ./build-setup.sh riscv-tools && \
    echo "source /home/chipyard/env.sh" >> ~/.bashrc 

RUN cd home/chipyard/sims/verilator && \
    make

WORKDIR /home

ENTRYPOINT ["xterm"]
