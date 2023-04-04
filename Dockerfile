FROM ubuntu:focal

# Handle timezone problems
ENV TZ=Europe/Rome
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone 

# Install graphical terminal
RUN apt update && apt install xterm dbus-x11 -y

# Install build necessities
RUN apt-get install build-essential cmake curl tar git verilator sudo -y 

# Install OpenROAD
RUN cd home && \
    git clone --recursive https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts && \
    cd OpenROAD-flow-scripts && \
    ./etc/DependencyInstaller.sh && \
    ./build_openroad.sh --local

ENV PATH /home/OpenROAD-flow-scripts/tools/OpenROAD/build/src:$PATH

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


# HAMMER {failed}

# RUN cd home/chipyard && \
#     echo "y" | conda create -c litex-hub --prefix ./.conda-sky130 open_pdks.sky130a=1.0.399_0_g63dbde9 && \
#     git clone https://github.com/rahulk29/sram22_sky130_macros ./sram22_sky130_macros && \
#     echo "y" | conda create -c litex-hub --prefix ./.conda-yosys yosys=0.27_4_gb58664d44  && \
#     echo "y" | conda create -c litex-hub --prefix ./.conda-signoff magic=8.3.376_0_g5e5879c netgen=1.5.250_0_g178b172 

# RUN cd home && \
#     apt install -y gcc g++ make qt5-default qttools5-dev libqt5xmlpatterns5-dev qtmultimedia5-dev libqt5multimediawidgets5 libqt5svg5-dev ruby ruby-dev python3 python3-dev libz-dev 

# RUN cd home && \
#     # curl -L -O https://github.com/KLayout/klayout/archive/refs/tags/v0.28.6.tar.gz && \
#     # tar hzxvf v0.28.6.tar.gz && \
#     git clone --recursive https://github.com/KLayout/klayout.git && \
#     cd klayout && \
#     # git checkout 0.28.6 && \
#     # ./setup.sh && \
#     ./build.sh -without-qtbinding -j4 -prefix /usr/bin/klayout

# RUN cd home && conda deactivate && conda deactivate && \
#     git clone --recursive https://github.com/The-OpenROAD-Project/OpenROAD.git && \
#     cd OpenROAD && \
#     ./etc/DependencyInstaller.sh && \
#     # ./etc/Build.sh 
#     mkdir build && \
#     cd build && \
#     cmake .. && \
#     make 

# RUN cd home/chipyard&& \
#     ./scripts/init-vlsi.sh sky130 openroad && \
#     cd vlsi && \
#     make buildfile tutorial=sky130-openroad


# Silicon Compiler
RUN cd home && \
    python3 --version && \
    sudo apt update && \
    sudo apt install -y python3-dev python3-pip python3-venv && \
    python3 -m venv  /home/venv && \
    echo " source /home/venv/bin/activate" >> ~/.bashrc 
     
# RUN pip install --upgrade pip && \
#     pip list && \
#     pip install --upgrade siliconcompiler && \
#     python -m pip show siliconcompiler

RUN cd home && \
    git clone -b v0.10.1 https://github.com/siliconcompiler/siliconcompiler && \
    cd siliconcompiler && \
    pip install -r requirements.txt && \
    python -m pip install -e .

RUN cd home/siliconcompiler/setup && \
    # apt install flex bison software-properties-common -y && \
    apt install -y lsb-release && \
    # /home/venv/bin/python3 -m pip install --upgrade pip install bambu && \
    # ./install-bluespec.sh && \
    # ./install-chisel.sh && \
    # ./install-ghdl.sh && \
    # ./install-icarus.sh && \
    # ./install-icepack.sh && \
    ./install-klayout.sh && \
    # ./install-magic.sh && \
    # ./install-netgen.sh && \
    # ./install-nextpnr.sh && \
    # ./install-openfpga.sh && \
    # ./install-openroad.sh && \
    # ./install-python.sh && \
    # ./install-rust.sh && \
    # ./install-surelog.sh && \
    # ./install-sv2v.sh && \
    # ./install-verilator.sh && \
    # ./install-xyce.sh && \
    # ./install-yosys.sh && \
    echo "finished"

ENV SCPATH=/home/siliconcompiler/siliconcompiler

# Surelog and yosys
RUN apt install -y build-essential cmake git pkg-config tclsh swig uuid-dev libgoogle-perftools-dev python3 python3-orderedmultidict python3-psutil python3-dev default-jre lcov && \
    cd home && \
    pip3 install orderedmultidict && \
    git config --global http.version HTTP/1.1 && \
    git clone https://github.com/alainmarcel/Surelog.git && \
    cd Surelog && \
    git submodule update --init --recursive && \
    make && make install

RUN cd home && \
    sudo apt-get install -y build-essential clang bison flex \
	libreadline-dev gawk tcl-dev libffi-dev git \
	graphviz xdot pkg-config python3 libboost-system-dev \
	libboost-python-dev libboost-filesystem-dev zlib1g-dev && \
    git clone https://github.com/YosysHQ/yosys.git && \
    cd yosys && make && make install

WORKDIR /home

ENTRYPOINT ["xterm"]
