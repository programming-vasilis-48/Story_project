# Cluster Setup Guide for Video Emotion Analysis

## Overview

Your emotion analysis currently takes 64 hours on a local machine. Using a computing cluster with GPU acceleration can reduce this to **2-6 hours** depending on the hardware configuration.

## Recommended Cluster Configuration

### **Option 1: GPU-Accelerated (Recommended)**
- **Platform**: Jupyter Lab or VS Code Server
- **Hardware**: 
  - GPU: NVIDIA V100, A100, or RTX 3080/4090 (16GB+ VRAM)
  - CPU: 8-16 cores
  - RAM: 32-64GB
  - Storage: 100GB+ (for video files and results)
- **Estimated Time**: 2-4 hours
- **Cost Estimate**: $5-15 per run (depending on cloud provider)

### **Option 2: High-CPU (Alternative)**
- **Platform**: Jupyter Lab or VS Code Server
- **Hardware**:
  - CPU: 32-64 cores (high-performance)
  - RAM: 64-128GB
  - Storage: 100GB+
- **Estimated Time**: 6-12 hours
- **Cost Estimate**: $10-25 per run

## Platform Recommendation: **Jupyter Lab**

**Why Jupyter Lab?**
- Interactive development and monitoring
- Easy file management and visualization
- Built-in terminal for command execution
- Can display plots and results inline
- Good for iterative analysis and debugging

## Setup Instructions

### 1. Cluster Resource Request

Request the following specifications:
```
Environment: Jupyter Lab
Python Version: 3.8+
GPU: NVIDIA V100/A100 (if available) with CUDA 11.0+
CPU: 16+ cores
RAM: 32GB+
Storage: 100GB
Duration: 6-8 hours (with buffer)
```

### 2. File Upload Strategy

**Option A: Direct Upload (Small datasets)**
- Upload the entire `video_analysis/` folder
- Upload `logs/sessions/` folder with videos

**Option B: Selective Upload (Recommended)**
- Upload only essential files:
  ```
  video_analysis/
  ├── core/
  ├── utils/
  ├── analysis/
  ├── visualization/
  ├── requirements.txt
  ├── run_analysis.py
  └── cluster_run.py (new file)
  
  logs/sessions/ (all video files)
  ```

### 3. Environment Setup

Create a setup script for the cluster:

```bash
# Install dependencies
pip install --upgrade pip
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install py-feat
pip install -r requirements.txt

# Verify GPU availability
python -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
python -c "import torch; print(f'GPU count: {torch.cuda.device_count()}')"
```

### 4. Optimized Analysis Script

Use the GPU-optimized version:

```python
# In Jupyter Lab cell or cluster_run.py
import os
os.environ['CUDA_VISIBLE_DEVICES'] = '0'  # Use first GPU

from video_analysis import run_complete_analysis

# Run with GPU acceleration
results = run_complete_analysis(save_individual=False)
print("Analysis completed!")
```

## Performance Optimizations for Cluster

### GPU Acceleration
- py-feat automatically uses GPU when available
- Batch processing is optimized for GPU memory
- Expected speedup: 10-20x faster than CPU

### Memory Management
- Process videos in batches to avoid memory overflow
- Use `save_individual=False` to reduce I/O overhead
- Clear GPU cache between large batches

### Parallel Processing
For multiple participants, you can run parallel jobs:

```python
# Split by participant for parallel processing
participants = ['andreea1', 'vasilis2', 'will3', 'vlad4', 'nikolaos5', 'alexadru6', 'amina7']

# Run each participant separately (can be parallelized)
for participant in participants:
    results = run_analysis_for_participant(participant)
```

## Expected Performance

| Configuration | Processing Time | Cost Estimate | Accuracy |
|---------------|----------------|---------------|----------|
| Local CPU | 64 hours | Free | High |
| Cluster CPU (32 cores) | 6-12 hours | $10-25 | High |
| Cluster GPU (V100) | 2-4 hours | $5-15 | High |
| Cluster GPU (A100) | 1-2 hours | $8-20 | High |

## Monitoring and Debugging

### In Jupyter Lab:
```python
# Monitor GPU usage
!nvidia-smi

# Check memory usage
import psutil
print(f"RAM usage: {psutil.virtual_memory().percent}%")

# Monitor progress
import time
start_time = time.time()
# ... run analysis ...
print(f"Elapsed time: {(time.time() - start_time)/3600:.2f} hours")
```

### Progress Tracking:
```python
# Add progress callbacks
def progress_callback(current, total):
    print(f"Progress: {current}/{total} ({current/total*100:.1f}%)")

# Use in analysis
results = run_complete_analysis(progress_callback=progress_callback)
```

## Data Transfer Strategy

### Upload (to cluster):
1. Compress video files: `tar -czf videos.tar.gz logs/sessions/`
2. Upload compressed archive
3. Extract on cluster: `tar -xzf videos.tar.gz`

### Download (from cluster):
1. Compress results: `tar -czf results.tar.gz video_analysis/results/`
2. Download compressed results
3. Extract locally

## Troubleshooting

### Common Issues:

1. **GPU Out of Memory**
   ```python
   # Reduce batch size in config
   DETECTION_CONFIG = {
       'batch_size': 16,  # Reduce from 32
       'max_frames': 500  # Limit frames per video
   }
   ```

2. **CUDA Version Mismatch**
   ```bash
   # Check CUDA version
   nvcc --version
   # Install compatible PyTorch
   pip install torch==1.13.1+cu117 -f https://download.pytorch.org/whl/torch_stable.html
   ```

3. **File Path Issues**
   ```python
   # Use absolute paths
   import os
   video_path = os.path.abspath('logs/sessions/')
   ```

## Cost Optimization Tips

1. **Use Spot/Preemptible Instances**: 50-80% cost reduction
2. **Monitor Usage**: Stop instances when not needed
3. **Batch Processing**: Process multiple experiments together
4. **Data Locality**: Keep data and compute in same region

## Alternative: Cloud Platforms

If your cluster doesn't have GPU access:

### Google Colab Pro
- Cost: $10/month
- GPU: T4, V100, A100
- Time: 2-6 hours
- Limitations: Session timeouts

### AWS SageMaker
- Cost: $1-3/hour
- GPU: Various options
- Time: 2-4 hours
- Scalable and reliable

### Azure Machine Learning
- Cost: $1-4/hour
- GPU: V100, A100
- Time: 2-4 hours
- Good integration tools

## Final Recommendations

1. **Start with Jupyter Lab** on your cluster with GPU
2. **Request 8 hours** of compute time (with buffer)
3. **Use GPU acceleration** if available (10-20x speedup)
4. **Monitor progress** and adjust batch sizes if needed
5. **Save intermediate results** to prevent data loss

The cluster approach should reduce your 64-hour analysis to **2-6 hours** with proper GPU acceleration, making it much more practical for iterative research and analysis.
