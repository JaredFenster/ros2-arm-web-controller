#!/usr/bin/env python3

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


class FloorImageProjector(Node):
    """Project RGB camera pixels onto a fixed ground plane and publish a colored point cloud."""

    def __init__(self) -> None:
        super().__init__("floor_image_projector")

        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("output_topic", "/camera/ground_projection")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("ground_z", 0.0)
        self.declare_parameter("pixel_step", 6)

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.world_frame = self.get_parameter("world_frame").value
        self.ground_z = float(self.get_parameter("ground_z").value)
        self.pixel_step = max(1, int(self.get_parameter("pixel_step").value))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._camera_info: Optional[CameraInfo] = None

        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, 10)
        self.create_subscription(Image, image_topic, self._image_cb, 10)
        self._cloud_pub = self.create_publisher(PointCloud2, self.output_topic, 10)

        self.get_logger().info(
            f"Projecting {image_topic} onto z={self.ground_z:.3f} in frame '{self.world_frame}' -> {self.output_topic}"
        )

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _image_cb(self, msg: Image) -> None:
        if self._camera_info is None:
            return

        if msg.height == 0 or msg.width == 0:
            return

        # Parse supported image encodings into RGB8 numpy array.
        rgb = self._to_rgb_array(msg)
        if rgb is None:
            return

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.world_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
            )
        except TransformException:
            return

        fx = self._camera_info.k[0]
        fy = self._camera_info.k[4]
        cx = self._camera_info.k[2]
        cy = self._camera_info.k[5]
        if fx == 0.0 or fy == 0.0:
            return

        h, w, _ = rgb.shape
        u = np.arange(0, w, self.pixel_step, dtype=np.float32)
        v = np.arange(0, h, self.pixel_step, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)
        uu_flat = uu.reshape(-1)
        vv_flat = vv.reshape(-1)

        x_cam = (uu_flat - cx) / fx
        y_cam = (vv_flat - cy) / fy
        z_cam = np.ones_like(x_cam)
        rays_cam = np.stack((x_cam, y_cam, z_cam), axis=1)

        q = tf_msg.transform.rotation
        r_mat = self._quat_to_rot_matrix(q.x, q.y, q.z, q.w)
        rays_world = rays_cam @ r_mat.T

        t = tf_msg.transform.translation
        origin = np.array([t.x, t.y, t.z], dtype=np.float32)

        dz = rays_world[:, 2]
        valid = np.abs(dz) > 1.0e-6
        valid &= ((self.ground_z - origin[2]) / dz) > 0.0

        if not np.any(valid):
            return

        t_hit = (self.ground_z - origin[2]) / dz[valid]
        pts = origin + rays_world[valid] * t_hit[:, None]

        u_idx = uu_flat[valid].astype(np.int32)
        v_idx = vv_flat[valid].astype(np.int32)
        colors = rgb[v_idx, u_idx, :]

        cloud = self._build_cloud(msg.header, pts, colors)
        self._cloud_pub.publish(cloud)

    def _to_rgb_array(self, msg: Image) -> Optional[np.ndarray]:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "rgb8":
            arr = data.reshape(msg.height, msg.width, 3)
            return arr
        if msg.encoding == "bgr8":
            arr = data.reshape(msg.height, msg.width, 3)
            return arr[:, :, ::-1]
        if msg.encoding == "rgba8":
            arr = data.reshape(msg.height, msg.width, 4)
            return arr[:, :, :3]
        if msg.encoding == "bgra8":
            arr = data.reshape(msg.height, msg.width, 4)
            return arr[:, :, 2::-1]
        if msg.encoding == "mono8":
            arr = data.reshape(msg.height, msg.width, 1)
            return np.repeat(arr, 3, axis=2)

        self.get_logger().warn(f"Unsupported image encoding '{msg.encoding}', expected rgb8/bgr8/rgba8/bgra8/mono8")
        return None

    def _build_cloud(self, header: Header, points_xyz: np.ndarray, colors_rgb: np.ndarray) -> PointCloud2:
        n = points_xyz.shape[0]
        rgb_uint32 = (
            (colors_rgb[:, 0].astype(np.uint32) << 16)
            | (colors_rgb[:, 1].astype(np.uint32) << 8)
            | colors_rgb[:, 2].astype(np.uint32)
        )
        rgb_float = rgb_uint32.view(np.float32)

        packed = np.empty(n, dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32), ("rgb", np.float32)])
        packed["x"] = points_xyz[:, 0].astype(np.float32)
        packed["y"] = points_xyz[:, 1].astype(np.float32)
        packed["z"] = points_xyz[:, 2].astype(np.float32)
        packed["rgb"] = rgb_float

        cloud = PointCloud2()
        cloud.header = Header(stamp=header.stamp, frame_id=self.world_frame)
        cloud.height = 1
        cloud.width = n
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * n
        cloud.is_dense = True
        cloud.data = packed.tobytes()
        return cloud

    @staticmethod
    def _quat_to_rot_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float32,
        )


def main() -> None:
    rclpy.init()
    node = FloorImageProjector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
