FROM ubuntu:lunar

RUN apt-get update && \
    apt-get install build-essential cmake curl tar git verilator -y 

RUN curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Mambaforge-$(uname)-$(uname -m).sh" && \
    (echo "\n"; echo "yes"; echo "/home/conda"; echo "yes") | bash Mambaforge-$(uname)-$(uname -m).sh
ENV PATH /home/conda/bin:$PATH
SHELL ["bash", "-c"]

RUN source /home/conda/etc/profile.d/conda.sh && conda init bash

RUN source /home/conda/etc/profile.d/conda.sh && \
    conda activate base && \
    conda install -n base conda-lock && \
    cd home && \
    git clone https://github.com/ucb-bar/chipyard.git

RUN source /home/conda/etc/profile.d/conda.sh && \
    conda activate base && \
    cd home/chipyard && \
    git checkout 1.8.1 && \
    ./build-setup.sh riscv-tools

RUN source /home/conda/etc/profile.d/conda.sh && \
    source /home/chipyard/env.sh && \
    cd /home/chipyard/sims/verilator && \
    make
    
WORKDIR /home

RUN echo "source /home/conda/etc/profile.d/conda.sh && source /home/chipyard/env.sh" >> ~/.bashrc 

CMD ["echo", "Alive!"]