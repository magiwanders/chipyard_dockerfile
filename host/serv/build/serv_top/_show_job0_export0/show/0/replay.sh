#!/bin/bash
export PATH="/home/venv/bin:/home/chipyard/.conda-env/riscv-tools/bin:/home/chipyard/software/firemarshal:/home/chipyard/bin:/home/chipyard/.conda-env/bin:/home/conda/condabin:/home/conda/bin:/home/OpenROAD-flow-scripts/tools/OpenROAD/build/src:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
klayout -nc -rm /home/siliconcompiler/siliconcompiler/tools/klayout/klayout_show.py -rd SC_ROOT=/home/siliconcompiler/siliconcompiler
