# Center-based 3D Cone Detection package for Formula Student Driverless Competition

## Installation
Install cudatoolkit 11.8 with runfile installation (assuming nvidia driver is already installed, check with command **nvidia-smi**)
```
wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run
sudo sh cuda_11.8.0_520.61.05_linux.run

# after the installation add this to your ~/.bashrc
export PATH="/usr/local/cuda-11.8/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH"
```

Install cuDNN 8.9.7 (example is for ubuntu), download the tar file first:
https://developer.nvidia.com/downloads/compute/cudnn/secure/8.9.7/local_installers/11.x/cudnn-linux-x86_64-8.9.7.29_cuda11-archive.tar.xz/
```
tar -xvf cudnn-linux-x86_64-8.9.7.29_cuda11-archive.tar.xz
sudo cp cudnn-*-archive/include/cudnn*.h /usr/local/cuda/include
sudo cp -P cudnn-*-archive/lib/libcudnn* /usr/local/cuda/lib64
sudo chmod a+r /usr/local/cuda/include/cudnn*.h /usr/local/cuda/lib64/libcudnn*
```

Restart of the system might needed, check correct installation with **nvcc --version**

Install venv for python (if not already installed).
```
sudo apt-get install python3-venv
```

Create your virtual enviroment (using conda is not advised because of ROS2)
```
python3 -m venv ~/lidar_detector
```

Install pytorch
```
# activate your virtual enviroment
source ~/lidar_detector/bin/activate  # you can deactivate it later with deactivate command

pip install torch==2.1.1 torchvision==0.16.1 torchaudio==2.1.1 --index-url https://download.pytorch.org/whl/cu118
```

OpenMMLab related installation:
```
pip install mmengine
pip install mmcv==2.1.0 -f https://download.openmmlab.com/mmcv/dist/cu118/torch2.1/index.html
pip install mmdet==3.3.0

# clone mmdetection3d
git clone https://github.com/jkk-research/mmdetection3d.git
cd mmdetection3d

pip install -e .
cd ..
pip install cumm-cu118
pip install spconv-cu118

# Install mmdeploy
git clone -b main https://github.com/open-mmlab/mmdeploy.git

pip install mmdeploy==1.3.1
# for inferencing with onnxruntime-gpu, tensorrt
pip install mmdeploy-runtime-gpu==1.3.1

# ----------- BELOW IS OPTIONAL -----------------
# Install tensorrt
wget https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/secure/8.6.1/tars/TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8.tar.gz
tar -xvzf TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-11.8.tar.gz

pip install TensorRT-8.6.1.6/python/tensorrt-8.6.1-cp310-none-linux_x86_64.whl
pip install pycuda
# after the installation add this to your ~/.bashrc
export LD_LIBRARY_PATH="/home/user/TensorRT-8.6.1.6/lib:$LD_LIBRARY_PATH"  # change path according to TensorRT-8.6.1.6 location

# Install onnxruntime-gpu
pip install onnxruntime-gpu==1.12.1

wget https://github.com/microsoft/onnxruntime/releases/download/v1.12.1/onnxruntime-linux-x64-gpu-1.12.1.tgz
tar -zxvf onnxruntime-linux-x64-gpu-1.12.1.tgz

# after the installation add this to your ~/.bashrc
export LD_LIBRARY_PATH="/home/user/onnxruntime-linux-x64-gpu-1.8.1/lib:$LD_LIBRARY_PATH"   # change path according to onnxruntime-linux-x64-gpu-1.8.1 location
```

Install dependency for ros2_numpy
```
pip install transforms3d
```

Build the package
```
colcon build --packages-select lidar_centerpoint_detector
```


## Acknowlegement
This project is not possible without multiple great opensourced codebases. We list some notable examples below.  
 
* [mmcv](https://github.com/open-mmlab/mmcv)
* [mmdetection3d](https://github.com/open-mmlab/mmdetection3d)
* [mmdeploy](https://github.com/open-mmlab/mmdeploy)

