#!/bin/bash

# Script to install dependencies for PyFeat

echo "Installing dependencies for PyFeat..."

# Update package lists
sudo apt-get update

# Install system dependencies
sudo apt-get install -y \
    python3-pip \
    python3-dev \
    python3-opencv \
    libopenblas-dev \
    liblapack-dev \
    gfortran

# Upgrade pip
pip3 install --upgrade pip

# Uninstall existing numpy and pandas to avoid conflicts
pip3 uninstall -y numpy pandas

# Install specific versions of numpy and pandas that are compatible with PyFeat
pip3 install numpy==1.20.0 pandas==1.3.0

# Install PyFeat dependencies
pip3 install torch torchvision
pip3 install matplotlib scikit-learn scikit-image

# Install PyFeat
pip3 install py-feat

echo "Installation complete!"
echo "Please restart your terminal or source your environment to ensure the changes take effect."
