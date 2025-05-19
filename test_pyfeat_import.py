#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test script to check PyFeat installation.
"""

import sys
print("Python version:", sys.version)
print("Python path:", sys.path)

try:
    print("Attempting to import feat...")
    import feat
    print("PyFeat imported successfully!")
    print("PyFeat version:", feat.__version__)
    
    print("\nAttempting to import Detector...")
    from feat import Detector
    print("Detector imported successfully!")
    
    print("\nInitializing detector...")
    detector = Detector(
        face_model="retinaface",
        landmark_model="mobilenet",
        au_model="xgb",
        emotion_model="resmasknet",
        facepose_model="img2pose"
    )
    print("Detector initialized successfully!")
    
except ImportError as e:
    print(f"ImportError: {e}")
    print("\nChecking for key dependencies...")
    
    try:
        import numpy
        print(f"NumPy version: {numpy.__version__}")
    except ImportError:
        print("NumPy not found")
    
    try:
        import pandas
        print(f"Pandas version: {pandas.__version__}")
    except ImportError:
        print("Pandas not found")
    
    try:
        import torch
        print(f"PyTorch version: {torch.__version__}")
    except ImportError:
        print("PyTorch not found")
        
except Exception as e:
    print(f"Other error: {e}")
    import traceback
    traceback.print_exc()
