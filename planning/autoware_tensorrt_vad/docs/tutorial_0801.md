# Tentative tutorials

## Setup

- [clone autoware and setup autoware](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/)
- copy autoware_tensorrt_vad package directory to `autoware/src/universe/autoware_universe/planning/autoware_tensorrt_vad`

## colcon build

```sh
cd autoware
source install/setup.bash
```

```sh
colcon build --packages-up-to autoware_tensorrt_vad
```

## download onnx files

Please download from [here](https://drive.google.com/drive/folders/1XemZvMOfHcp0BYssCGLhy4f62v4p0Q7X?usp=drive_link) and put them in `autoware_tensorrt_vad/data` directory.

## launch

```sh
ros2 launch autoware_tensorrt_vad vad.launch.xml
```
