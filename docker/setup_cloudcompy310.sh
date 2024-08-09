#!/bin/bash

# Update Path to ensure conda is on path with different user
export PATH="/conda/condabin/:${PATH}"

# miniconda and mamba
source "/conda/etc/profile.d/conda.sh"
# For mamba support also run the following command
source "/conda/etc/profile.d/mamba.sh"

# Activate environment with access to cloud com py
source /CloudComPy/bin/condaCloud.sh activate CloudComPy310