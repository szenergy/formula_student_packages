# Dockerfile for containerzation.
Using this dockerfile it is possible to launch the whole system (ROS2 backend) in a virtually separated container. This should simplify installation and launch (but may come with its own disadvantages).  

> [!NOTE]  
> <small>Note: This is a single-**architecture** docker image, only meant for **x86-64 / AMD64** (like most laptops and desktop computers)... Other systems however (e.g. Jetson) may require an AArch64 / ARM64 docker image.</small>  

The default **frontend** for this setup is **Foxglove Studio** [ https://foxglove.dev/download ] and it needs to be downloaded, installed and configured (customized) separately. Other software (such as RViZ*) can also be used instead, but their scope/access may be limited to within the container.  
<sub><sup>*RViZ may need `export LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:$LD_LIBRARY_PATH` first, for graphics.</sup></sub>

## Extra hardware requirements:
- **Cuda**-compatible **GPU** (NVidia)
    - <sup><sub>*tiny note: GPU acceleration is assumed, but ultimately not required - certain sections need to be modified otherwise (pretty much everything cuda-related, so... about half of the dockerfile)</sub></sup>
- **Docker** Engine
    - Docker Desktop app (and possibly WSL2 setup) for Windows or Docker Engine (CLI) for Linux

## Additional requirements for WSL (if using Windows), for the GUI:
- X11 or other **window system** that wsl can connect to in order to create windows in Windows.
    - *Installation may not be needed.*
    - You can run xclock in a terminal to see if it works. ( `sudo apt install x11-apps && xclock` )
    - <sup><sub>( here is a guide that *should* work otherwise: https://www.guide2wsl.com/x11/ )</sub></sup>

## External software requirements (host machine software)
- **MMDetection3D**
    - no install required here, just the raw files
    - in the same directory as the dockerfile run:  
```git clone https://github.com/jkk-research/mmdetection3d/```  
    - (this requires having Git installed - or download this directly from the link)
- *Foxglove Studio* (as frontend) [Optional]
    - https://foxglove.dev/download


## Build and run:
### Build command:
( With MMDetection3D in the dockerfile directory )
```
docker build -t formula_student_stack_image .
```
<small>( = in your dockerfile's directory - more generally: `docker build -t <chooseAnImageName> <yourPath>` )</small>

### Native Linux run command:
If you want to create any window (e.g. for GUI) from WITHIN the container, first run:
```
xhost +local:docker
```
<sup> = *grant X server (window system) access*</sup>

Running the container:
```
docker run -it --rm --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/.Xauthority:/root/.Xauthority:r --name formula_student_stack formula_student_stack_image
```
<details>
<summary>Breakdown in detail</summary>

- `run` = run a container...
- `-it` = with terminal access (in interactive mode [STDIN = kept open], providing a virtual terminal [TTY] )
- `--rm` = but remove container ENTIRELY after execution (after closing the shell that this command opens)
    - you may want to remove this for a non-volatile/permamnent container.
- `--net=host` = using the same network as the host machine, directly
- `-e DISPLAY=$DISPLAY` = setting environmental variable `DISPLAY` as host's default display (for window rendering)
- `-v` = \[virtually\] mounting:
    - host's `/tmp/.X11-unix` as `/tmp/.X11-unix` in-container
        - host system's window access / X11 Unix socket forwarding
    - host's `~/.Xauthority` as `/root/.Xauthority` in-container \[`:r` = with read-only access\]
        - X \[window\] server authentication tokens (ensuring access, may not be needed)
        - (assumes the container runs as root)
        - <sub><sup>(you may change `:r` to `:rw` for read-write access but it should work without that)</sup></sub>
- `--name` = naming it: `formula_student_stack`
- using `formula_student_stack_image` as the base image ("blueprint") to create the container from
</details>

### WSL (Windows) run command:
```
docker run -it --rm --net=host -e DISPLAY=host.docker.internal:0 -e LIBGL_ALWAYS_INDIRECT=0 -e XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" -v /tmp/.X11-unix:/tmp/.X11-unix --privileged --gpus all --name formula_student_stack formula_student_stack_image
```
<details>
<summary>Breakdown in detail</summary>

- `run` = run a container...
- `-it` = with terminal access (in interactive mode [STDIN = kept open], providing a virtual terminal [TTY] )
- `--rm` = but remove container ENTIRELY after execution (after closing the shell that this command opens)
    - you may want to remove this for a non-volatile/permamnent container.
- `--net=host` = using the same network as the host machine, directly
- `-e` = setting environmental variable(s):
    - `DISPLAY=host.docker.internal:0` (display setup for window rendering)
    - `LIBGL_ALWAYS_INDIRECT=0` (graphics/OpenGL forwarding)
    - `XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR"` (ensuring GUI resource access)
- `-v` = \[virtually\] mounting: `/tmp/.X11-unix` as `/tmp/.X11-unix` (host system's window access / X11 Unix socket forwarding)
- `--privileged` = with elevated privilages (admin/root) - ensure hardware (GPU) access rights
- `--gpus all` = enabling GPU access
- `--name` = naming it: `formula_student_stack`
- using `formula_student_stack_image` as the base image ("blueprint") to create the container from
</details>

### To start another shell in the running container you can just:
```
docker exec -it formula_student_stack bash
```  
> <small> - meaning: execute command 'bash' (start terminal shell), with interactive terminal access, in the container named 'formula student_stack'</small>  
    <sub>*(note: if no "--name" has been specified during build then docker automatically assigns a random one which can be referenced similarly, once found)*</sub>
