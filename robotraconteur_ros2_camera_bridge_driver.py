import cv2
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
import RobotRaconteurCompanion as RRC
import argparse
import sys
import platform
import threading
import numpy as np
from RobotRaconteurCompanion.Util.InfoFileLoader import InfoFileLoader
from RobotRaconteurCompanion.Util.DateTimeUtil import DateTimeUtil
from RobotRaconteurCompanion.Util.SensorDataUtil import SensorDataUtil
from RobotRaconteurCompanion.Util.AttributesUtil import AttributesUtil
from RobotRaconteurCompanion.Util.IdentifierUtil import IdentifierUtil
from contextlib import suppress

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
import uuid


class CameraImpl(object):

    def __init__(self, ros_topic, camera_info, ros_node):


        self._imaging_consts = RRN.GetConstants('com.robotraconteur.imaging')
        self._image_consts = RRN.GetConstants('com.robotraconteur.image')
        self._image_type = RRN.GetStructureType('com.robotraconteur.image.Image')
        self._image_info_type = RRN.GetStructureType('com.robotraconteur.image.ImageInfo')
        self._compressed_image_type = RRN.GetStructureType('com.robotraconteur.image.CompressedImage')
        self._date_time_utc_type = RRN.GetPodDType('com.robotraconteur.datetime.DateTimeUTC')
        self._isoch_info = RRN.GetStructureType('com.robotraconteur.device.isoch.IsochInfo')
        self._camera_state_type = RRN.GetStructureType('com.robotraconteur.imaging.CameraState')
        self._fps = 0
        self._camera_info = camera_info
        self._date_time_util = DateTimeUtil(RRN)
        self._sensor_data_util = SensorDataUtil(RRN)
        self._identifier_util = IdentifierUtil(RRN)
        self._seqno = 0

        self._state_timer = None

        self._wires_ready = False

        self._ros_node = ros_node
        self._ros_cv_bridge = CvBridge()

        self._image_sub = ros_node.create_subscription(ROSImage, ros_topic, self._image_cb, 10)

        self._current_frame = None

    def RRServiceObjectInit(self, ctx, service_path):
        self._downsampler = RR.BroadcastDownsampler(ctx)
        self._downsampler.AddPipeBroadcaster(self.frame_stream)
        self._downsampler.AddPipeBroadcaster(self.frame_stream_compressed)
        self._downsampler.AddPipeBroadcaster(self.preview_stream)
        self._downsampler.AddWireBroadcaster(self.device_clock_now)
        self.frame_stream.MaxBacklog = 2
        self.frame_stream_compressed.MaxBacklog = 2
        self.preview_stream.MaxBacklog = 2

        # TODO: Broadcaster peek handler in Python
        self.device_clock_now.PeekInValueCallback = lambda ep: self._date_time_util.FillDeviceTime(
            self._camera_info.device_info, self._seqno)

        self._state_timer = RRN.CreateTimer(0.05, self._state_timer_cb)
        self._state_timer.Start()

        self._wires_ready = True

    def _close(self):
        self._state_timer.TryStop()
        self._image_sub.destroy()

    def _state_timer_cb(self, timer_evt):
        s = self._camera_state_type()
        self._seqno += 1
        s.ts = self._date_time_util.TimeSpec3Now()
        s.seqno = self._seqno
        flags = self._imaging_consts["CameraStateFlags"]["ready"]
        flags |= self._imaging_consts["CameraStateFlags"]["streaming"]
        s.state_flags = flags

        self.camera_state.OutValue = s

    @property
    def device_info(self):
        return self._camera_info.device_info

    @property
    def camera_info(self):
        return self._camera_info

    def _cv_mat_to_image(self, mat):

        is_mono = False
        if (len(mat.shape) == 2 or mat.shape[2] == 1):
            is_mono = True

        image_info = self._image_info_type()
        image_info.width = mat.shape[1]
        image_info.height = mat.shape[0]
        if is_mono:
            image_info.step = mat.shape[1]
            image_info.encoding = self._image_consts["ImageEncoding"]["mono8"]
        else:
            image_info.step = mat.shape[1] * 3
            image_info.encoding = self._image_consts["ImageEncoding"]["bgr888"]
        image_info.data_header = self._sensor_data_util.FillSensorDataHeader(self._camera_info.device_info, self._seqno)

        image = self._image_type()
        image.image_info = image_info
        image.data = mat.reshape(mat.size, order='C')
        return image

    def _cv_mat_to_compressed_image(self, mat, quality=100):

        is_mono = False
        if (len(mat.shape) == 2 or mat.shape[2] == 1):
            is_mono = True

        image_info = self._image_info_type()
        image_info.width = mat.shape[1]
        image_info.height = mat.shape[0]

        image_info.step = 0
        image_info.encoding = self._image_consts["ImageEncoding"]["compressed"]
        image_info.data_header = self._sensor_data_util.FillSensorDataHeader(self._camera_info.device_info, self._seqno)

        image = self._compressed_image_type()
        image.image_info = image_info
        res, encimg = cv2.imencode(".jpg", mat, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        assert res, "Could not compress frame!"
        image.data = encimg
        return image

    def capture_frame(self):
        if self._current_frame is None:
            raise RR.OperationFailedException("No current frame")
        mat = self._current_frame
        return self._cv_mat_to_image(mat)

    def capture_frame_compressed(self):
        if self._current_frame is None:
            raise RR.OperationFailedException("No current frame")
        mat = self._current_frame
        return self._cv_mat_to_compressed_image(mat)

    def trigger(self):
        raise RR.NotImplementedException("Not available on this device")

    def _image_cb(self, ros_msg):

        self._seqno += 1
        

        mat = self._ros_cv_bridge.imgmsg_to_cv2(ros_msg, desired_encoding="bgr8")
        self._current_frame = mat

        if not self._wires_ready:
            return

        self.frame_stream.AsyncSendPacket(self._cv_mat_to_image(mat), lambda: None)
        self.frame_stream_compressed.AsyncSendPacket(self._cv_mat_to_compressed_image(mat), lambda: None)
        self.preview_stream.AsyncSendPacket(self._cv_mat_to_compressed_image(mat, 70), lambda: None)
        device_now = self._date_time_util.FillDeviceTime(self._camera_info.device_info, self._seqno)
        self.device_clock_now.OutValue = device_now

    def start_streaming(self):
        pass

    def stop_streaming(self):
        pass

    @property
    def isoch_downsample(self):
        return self._downsampler.GetClientDownsample(RR.ServerEndpoint.GetCurrentEndpoint())

    @isoch_downsample.setter
    def isoch_downsample(self, value):
        return self._downsampler.SetClientDownsample(RR.ServerEndpoint.GetCurrentEndpoint(), value)

    @property
    def isoch_info(self):
        ret = self._isoch_info()
        ret.update_rate = 0
        ret.max_downsample = 100
        ret.isoch_epoch = np.zeros((1,), dtype=self._date_time_utc_type)

    @property
    def capabilities(self):
        return 0x1 | 0x2 | 0x4

    def getf_param(self, name):
        raise RR.InvalidOperationException("Parameter not found")

    def setf_param(self, name, value):
        raise RR.InvalidOperationException("Parameter not found")

    @property
    def param_info(self):

        return dict()


def main():

    rclpy.init(args=sys.argv)

    parser = argparse.ArgumentParser(description="OpenCV based camera driver service for Robot Raconteur")
    parser.add_argument("--ros-topic", type=str, default="/image_raw", required=True, help="ROS topic for camera (default is /image_raw)")
    parser.add_argument("--camera-info-file", type=argparse.FileType('r'),
                        default=None, required=True, help="Camera info file (required)")
    

    args, _ = parser.parse_known_args()

    rr_args = ["--robotraconteur-jumbo-message=true"] + sys.argv

    # RRN.RegisterServiceTypesFromFiles(['com.robotraconteur.imaging'],True)
    RRC.RegisterStdRobDefServiceTypes(RRN)

    with args.camera_info_file:
        camera_info_text = args.camera_info_file.read()

    info_loader = InfoFileLoader(RRN)
    camera_info, camera_ident_fd = info_loader.LoadInfoFileFromString(
        camera_info_text, "com.robotraconteur.imaging.camerainfo.CameraInfo", "camera")

    attributes_util = AttributesUtil(RRN)
    camera_attributes = attributes_util.GetDefaultServiceAttributesFromDeviceInfo(camera_info.device_info)

    ros_node = rclpy.create_node("robotraconteur_ros2_camera_bridge_driver_" + str(uuid.uuid4()).replace('-', ''))

    camera = CameraImpl(args.ros_topic, camera_info, ros_node)

    with RR.ServerNodeSetup("com.robotraconteur.imaging.camera", 59824, argv=rr_args):

        service_ctx = RRN.RegisterService("camera", "com.robotraconteur.imaging.Camera", camera)
        service_ctx.SetServiceAttributes(camera_attributes)

        # Wait for exit
        print("Press Ctrl-C to quit...")
        try:
            rclpy.spin(ros_node)
        except KeyboardInterrupt:
            pass

        camera._close()


if __name__ == "__main__":
    main()
