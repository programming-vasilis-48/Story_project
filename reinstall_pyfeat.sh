#!/bin/bash

# Script to reinstall PyFeat and its dependencies correctly

echo "Reinstalling PyFeat and its dependencies..."

# Uninstall existing PyFeat and potential conflicting packages
echo "Uninstalling existing packages..."
pip uninstall -y py-feat
pip uninstall -y numpy pandas torch torchvision

# Install specific versions of dependencies known to work with PyFeat
echo "Installing compatible dependencies..."
pip install numpy==1.20.0
pip install pandas==1.3.0
pip install torch==1.10.0 torchvision==0.11.0
pip install matplotlib==3.5.0 scikit-learn==1.0.1 scikit-image==0.18.3

# Install PyFeat
echo "Installing PyFeat..."
pip install py-feat

# Test the installation
echo "Testing PyFeat installation..."
python3 -c "from feat import Detector; print('PyFeat successfully installed!')"

echo "Installation complete!"
echo "If you still have issues, try running the test_pyfeat_import.py script for more detailed diagnostics."
