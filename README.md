# Robot Raconteur ROS 2 Camera Bridge Driver

The Robot Raconteur ROS 2 Camera Bridge Driver provides a standard Robot Raconteur `com.robotraconteur.imaging.Camera` service interface for ROS 2 camera drivers. The driver uses a ROS 2 "raw image" topic as the image source. The Robot Raconteur service provides the standard image capture and streaming functions. See the normal [Robot Raconteur Camera Driver]() for more information and examples using the `com.robotraconteur.imaging.Camera` service interface. Note that this driver listens on port 59824 so the URL will need to be modified. See the connection information below.

## Connection Info

The default connection information is as follows. These details may be changed using `--robotraconteur-*` command
line options when starting the service. Also see the
[Robot Raconteur Service Browser](https://github.com/robotraconteur/RobotRaconteur_ServiceBrowser) to detect
services on the network.

- URL: `rr+tcp://localhost:59824?service=camera`
- Device Name: `camera` or device name in the configuration file
- Node Name: `robotraconteur_ros2_camera_bridge_driver`
- Service Name: `camera`
- Root Object Type:
  - `com.robotraconteur.imaging.Camera`

## Installation

This driver requires ROS 2 to be installed. After the normal ROS 2 apt setup, run the following commands, run the following setup. Replace `jazzy` with the ROS 2 distro you are using.

```bash
sudo apt install ros-jazzy-ros-base ros-jazzy-v4l2-camera -y
```

This driver must be installed using a Python virtual environment. The following commands will install the driver in a Python virtual environment. Run inside the repository root directory.

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv venv --system-site-packages --symlinks
source venv/bin/activate
python3 -m pip install .
```

Use `python3 -m pip install -e .` to install in editable mode.

## Running the Driver

Begin by running the ROS camera driver in a terminal.

```bash
source /opt/ros/jazzy/setup.bash
ros2 run v4l2_camera v4l2_camera_node
```

Then run the Robot Raconteur ROS 2 Camera Bridge Driver in another terminal.

```bash
source /opt/ros/jazzy/setup.bash
source venv/bin/activate
robotraconteur-ros2-camera-bridge-driver --ros-topic=/image_raw --camera-info-file=config/generic_webcam_720p_default_camera_info.yml
```

Replace `--ros-topic=` with the ROS 2 topic name.

The `--camera-info-file=` is required, and is used to populate the metadata for the camera driver. This includes
the `device_info` and `camera_info` properties provided by the service. Default config files are provided
for 720p and 1080p webcams. See the [CameraInfo](https://github.com/robotraconteur/robotraconteur_standard_robdef/blob/master/docs/info_files/camerainfo.m) documentation for more information.

## Example Clients

See the [standard camera driver examples](https://github.com/robotraconteur-contrib/robotraconteur_camera_driver/tree/master/examples). Note that the URL will need to be modified to connect to the camera bridge driver.
See the connection information above.

A simple Python example is provided below.

```python
# Simple example Robot Raconteur standard camera client
# This program will capture a single frame and
# display it.

from RobotRaconteur.Client import *
from RobotRaconteurCompanion.Util.ImageUtil import ImageUtil

import cv2
import sys
import traceback
import argparse
import numpy as np
import traceback


def main():

    # URL for connecting to the camera
    url = 'rr+tcp://localhost:59824?service=camera'
    if (len(sys.argv) >= 2):
        url = sys.argv[1]

    # Connect to the camera
    cam = RRN.ConnectService(url)

    # Create an ImageUtil object to help with image conversion
    image_util = ImageUtil(RRN, cam)

    # Capture the frame from the camera, returns in raw format
    raw_frame = cam.capture_frame()

    # Convert raw_img to opencv format using companion library ImageUtil
    current_frame = image_util.image_to_array(raw_frame)

    cv2.namedWindow("Image")

    cv2.imshow("Image", current_frame)
    cv2.waitKey()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

```

## License

License: Apache 2.0

Author: John Wason, PhD

## Acknowledgment

This work was supported in part by the Advanced Robotics for Manufacturing ("ARM") Institute under Agreement Number W911NF-17-3-0004 sponsored by the Office of the Secretary of Defense. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of either ARM or the Office of the Secretary of Defense of the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes, notwithstanding any copyright notation herein.

![](https://github.com/robotraconteur/robotraconteur/blob/master/docs/figures/arm_logo.jpg?raw=true)
