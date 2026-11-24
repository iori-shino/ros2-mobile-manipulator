#!/usr/bin/env python3
"""Classify three colored cubes and project their front bottom edges onto a level table."""

import math
from dataclasses import dataclass
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Point, PointStamped
from image_geometry import PinholeCameraModel
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2D, Detection2DArray, Detection3D, Detection3DArray
from vision_msgs.msg import ObjectHypothesisWithPose


HUE_BANDS = {'red': ((0, 10), (170, 179)), 'blue': ((95, 135),), 'yellow': ((17, 39),)}
DRAW_COLORS = {'red': (80, 80, 255), 'blue': (255, 190, 75), 'yellow': (30, 245, 245)}
KERNEL = np.ones((3, 3), dtype=np.uint8)
OUTPUT_FRAME = 'arm_base_link'


@dataclass(frozen=True)
class Blob:
    color: str
    rect: tuple
    base_pixel: tuple
    fill: float


def find_cubes(image, min_area):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    height, width = hsv.shape[:2]
    for color, bands in HUE_BANDS.items():
        mask = np.zeros((height, width), np.uint8)
        for low, high in bands:
            mask |= cv2.inRange(hsv, (low, 95, 45), (high, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            area = cv2.contourArea(contour)
            if area < min_area:
                break
            x, y, w, h = cv2.boundingRect(contour)
            fill = area / (w * h)
            if not (0 < x and x+w < width and 0 < y and y+h < height
                    and .60 <= w/h <= 1.65 and fill >= .65
                    and area >= .88 * cv2.contourArea(cv2.convexHull(contour))):
                continue
            # With a frontal, level view looking down onto the tabletop, the
            # lowest long horizontal span belongs to the near face's bottom edge.
            lowest = mask[y+h-1, x:x+w]
            occupied = np.flatnonzero(lowest)
            bottom = float(y+h)
            if len(occupied) < .55*w:
                # A small docking yaw can split the visible edge across two rows.
                occupied = np.flatnonzero(lowest | mask[y+h-2, x:x+w])
                if len(occupied) < .55*w:
                    continue
                bottom -= 1. - np.count_nonzero(lowest)/len(occupied)
            base_pixel = (float(x + (occupied[0]+occupied[-1]+1)/2), bottom)
            yield Blob(color, (x, y, w, h), base_pixel, fill)
            break


def project_center(blob, model, camera_in_plane, arm_from_plane, table_height, cube_size):
    ray = model.projectPixelTo3dRay(model.rectifyPoint(blob.base_pixel))
    origin = camera_in_plane.transform.translation
    end = do_transform_point(PointStamped(point=Point(x=ray[0], y=ray[1], z=ray[2])),
                             camera_in_plane).point
    dx, dy, dz = end.x-origin.x, end.y-origin.y, end.z-origin.z
    if dz >= -.01 or origin.z <= table_height:
        return None
    scale = (table_height-origin.z) / dz
    half = cube_size / 2
    # The near face is normal to the table frame's X axis. Offset behind
    # the face and above the tabletop to obtain the cube's geometric center.
    center = PointStamped(point=Point(x=origin.x+scale*dx+half,
                                     y=origin.y+scale*dy, z=table_height+half))
    return do_transform_point(center, arm_from_plane).point


def hypothesis(blob):
    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = blob.color
    result.hypothesis.score = blob.fill
    return result


class CubeLocator(Node):
    def __init__(self):
        super().__init__('color_cube_locator')
        self.declare_parameter('plane_frame', 'base_footprint')
        self.declare_parameter('table_height_m', .086)
        self.declare_parameter('cube_size_m', .035)
        self.declare_parameter('min_area_px', 100.)
        self.declare_parameter('image_timeout_sim_s', .5)
        self.declare_parameter('image_timeout_wall_s', 5.)
        self.plane_frame = self.get_parameter('plane_frame').value
        self.table_height = self.get_parameter('table_height_m').value
        self.cube_size = self.get_parameter('cube_size_m').value
        self.min_area = self.get_parameter('min_area_px').value
        self.timeout_sim = self.get_parameter('image_timeout_sim_s').value
        self.timeout_wall = self.get_parameter('image_timeout_wall_s').value
        self.camera = PinholeCameraModel()
        self.calibration = None
        self.bridge = CvBridge()
        self.transforms = Buffer()
        self.listener = TransformListener(self.transforms, self)
        self.last_stamp = None
        self.last_wall = -math.inf
        self.found_targets = False
        self.optical_frame = 'camera_optical_frame'
        self.pixels = self.create_publisher(Detection2DArray, '/m3/detections_2d', 5)
        self.meters = self.create_publisher(Detection3DArray, '/m3/detections_3d', 5)
        self.debug = self.create_publisher(Image, '/m3/debug_image', qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_timer(.2, self.expire, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_info(self, info):
        if info.k[0] > 0 and info.k[4] > 0 and info.p[0] > 0 and info.p[5] > 0:
            self.calibration = (info.header.frame_id, info.width, info.height)
            self.camera.fromCameraInfo(info)

    def publish(self, stamp, pixels, meters):
        self.pixels.publish(Detection2DArray(
            header=Header(stamp=stamp, frame_id=self.optical_frame), detections=pixels))
        self.meters.publish(Detection3DArray(
            header=Header(stamp=stamp, frame_id=OUTPUT_FRAME), detections=meters))
        self.found_targets = bool(pixels or meters)

    def expire(self):
        now = self.get_clock().now()
        if self.last_stamp is None:
            return
        age = (now-self.last_stamp).nanoseconds / 1e9
        if age < -self.timeout_sim:
            self.last_stamp = None
        if self.found_targets and (age > self.timeout_sim or age < -self.timeout_sim
                                   or time.monotonic()-self.last_wall > self.timeout_wall):
            self.publish(now.to_msg(), [], [])

    def on_image(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now()-stamp).nanoseconds / 1e9
        if (not stamp.nanoseconds or not -.25 <= age <= self.timeout_sim
                or (self.last_stamp is not None and stamp <= self.last_stamp)):
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warning(f'Invalid RGB image: {exc}', throttle_duration_sec=5.)
            return
        self.last_stamp = stamp
        self.last_wall = time.monotonic()
        self.optical_frame = msg.header.frame_id
        blobs = list(find_cubes(image, self.min_area))
        projected = {}
        if blobs and self.calibration == (msg.header.frame_id, msg.width, msg.height):
            try:
                camera_in_plane = self.transforms.lookup_transform(self.plane_frame, msg.header.frame_id, stamp)
                arm_from_plane = self.transforms.lookup_transform(OUTPUT_FRAME, self.plane_frame, stamp)
                projected = {blob.color: project_center(blob, self.camera, camera_in_plane,
                                                       arm_from_plane, self.table_height, self.cube_size)
                             for blob in blobs}
            except TransformException as exc:
                self.get_logger().warning(f'TF unavailable at image stamp: {exc}', throttle_duration_sec=5.)

        pixels, meters = [], []
        for blob in blobs:
            x, y, w, h = blob.rect
            detection = Detection2D(header=msg.header, id=blob.color, results=[hypothesis(blob)])
            detection.bbox.center.position.x = float(x+w/2)
            detection.bbox.center.position.y = float(y+h/2)
            detection.bbox.size_x, detection.bbox.size_y = float(w), float(h)
            pixels.append(detection)
            point = projected.get(blob.color)
            if point is not None:
                result = hypothesis(blob)
                result.pose.pose.position = point
                item = Detection3D(header=Header(stamp=msg.header.stamp, frame_id=OUTPUT_FRAME),
                                   id=blob.color, results=[result])
                item.bbox.center.position = point
                item.bbox.size.x = item.bbox.size.y = item.bbox.size.z = self.cube_size
                meters.append(item)

        self.publish(msg.header.stamp, pixels, meters)
        if self.debug.get_subscription_count():
            self.draw_debug(image, msg.header, blobs, projected)

    def draw_debug(self, image, header, blobs, projected):
        h = image.shape[0]
        canvas = cv2.copyMakeBorder(image, 0, 66, 0, 0, cv2.BORDER_CONSTANT, value=(24, 24, 24))
        for index, blob in enumerate(blobs):
            x, y, w, height = blob.rect
            color = DRAW_COLORS[blob.color]
            cv2.rectangle(canvas, (x, y), (x+w-1, y+height-1), color, 1)
            cv2.putText(canvas, blob.color, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, .42, color, 1)
            cv2.drawMarker(canvas, tuple(map(round, blob.base_pixel)), color, cv2.MARKER_CROSS, 9)
            point = projected.get(blob.color)
            label = f'{blob.color} ({x+w/2:g},{y+height/2:g})'
            label += (f'  ({point.x:.3f},{point.y:.3f},{point.z:.3f}) m'
                      if point is not None else '  no 3D position')
            cv2.putText(canvas, label, (5, h+16+index*18), cv2.FONT_HERSHEY_SIMPLEX, .41, color, 1)
        if not blobs:
            cv2.putText(canvas, 'No cube detected', (5, h+20), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, (215, 215, 215), 1)
        self.debug.publish(self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8', header=header))


def main():
    rclpy.init()
    node = CubeLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
