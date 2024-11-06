# qrb_ros_yolo_processor
qrb_ros_yolo_processor provides ros nodes to execute pre/post-process for Yolo model

## Overview
qrb_ros_yolo_processor provides ros nodes to execute pre/post-process for Yolo model

## System Requirements

- ROS 2 Humble and later

## Quickstart

### Code Sync

Currently, we only support build with QCLINUX SDK.

1. Setup QCLINUX SDK environments follow this document: [Set up the cross-compile environment](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/develop-your-first-application_6.html?product=1601111740013072&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK&state=releasecandidate)

2. Create `ros_ws` directory in `<qirp_decompressed_workspace>/qirp-sdk/`

   ```bash
   mkdir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws
   ```

3. Clone this repository and dependencies under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`

   ```bash
   cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws

   ## current repository
   git clone https://github.qualcomm.com/QUIC-QRB-ROS/qrb_ros_yolo_processor.git

   ## dependencies
   git clone https://github.com/quic-qrb-ros/qrb_ros_vision_msgs.git
   ```

### Build

   ```bash
   export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
   export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages

   colcon build --merge-install --cmake-args \
      -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
      -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
      -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
      -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
      -DBUILD_TESTING=OFF
   ```
### Run

1. Set up the environment on your device:

   ```bash
   ssh root@[ip-addr]
   (ssh) export HOME=/opt
   (ssh) source /opt/qcom/qirp-sdk/qirp-setup.sh
   (ssh) source /usr/bin/ros_setup.bash
   ```

2. use launch file to run your inference pipeline

   ```bash
   (ssh) ros2 launch ${package_name} ${launch-file}
   ```

## Resources


## Contributions

Thanks for your interest in contributing to qrb_ros_yolo_processor! Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## License

qrb_ros_yolo_processor is licensed under the BSD 3-clause "New" or "Revised" License.

Check out the [LICENSE](LICENSE) for more details.

