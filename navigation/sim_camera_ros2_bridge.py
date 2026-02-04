#!/usr/bin/env python3
# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""
Simulation Camera to ROS2 Bridge

Reads RGB and depth images from Isaac Lab shared memory and publishes
to ROS2 topics matching the Intel RealSense D435 driver format.

This enables seamless switching between simulation and real robot:
- Simulation: This bridge reads from shared memory, publishes to ROS2
- Real robot: RealSense driver publishes directly to the same ROS2 topics

Published Topics (matching RealSense driver):
    /camera/color/image_raw (sensor_msgs/Image) - RGB image
    /camera/depth/image_rect_raw (sensor_msgs/Image) - Depth image (16UC1, mm)
    /camera/color/camera_info (sensor_msgs/CameraInfo) - RGB camera intrinsics
    /camera/depth/camera_info (sensor_msgs/CameraInfo) - Depth camera intrinsics
    /camera/depth/color/points (sensor_msgs/PointCloud2) - Colored point cloud

Usage:
    # In navigation container
    python3 -m navigation.sim_camera_ros2_bridge
    
    # Or with custom settings
    CAMERA_FRAME=camera_link python3 -m navigation.sim_camera_ros2_bridge
"""

import sys
import os
import time
import numpy as np

# Add project root to path for shared memory utils
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
    from std_msgs.msg import Header
    from builtin_interfaces.msg import Time
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ERROR: ROS2 not available. Install ROS2 Humble.")

try:
    from tools.shared_memory_utils import MultiImageReader, DepthImageReader
    SHM_AVAILABLE = True
except ImportError:
    SHM_AVAILABLE = False
    print("WARNING: Shared memory utils not available")


class SimCameraROS2Bridge(Node):
    """Bridge camera data from Isaac Lab shared memory to ROS2 topics
    
    Publishes to topics matching RealSense driver for seamless sim/real switching.
    """
    
    def __init__(
        self,
        node_name: str = 'sim_camera_bridge',
        frame_id: str = 'camera_link',
        publish_rate: float = 30.0,
        image_width: int = 640,
        image_height: int = 480,
    ):
        super().__init__(node_name)
        
        self.frame_id = frame_id
        self.image_width = image_width
        self.image_height = image_height
        
        # Camera intrinsics (approximate Intel RealSense D435)
        # D435 depth: fx=fy~=382, cx=319.5, cy=239.5 for 640x480
        self.fx = 382.0
        self.fy = 382.0
        self.cx = 319.5
        self.cy = 239.5
        
        # QoS for camera topics - use RELIABLE for compatibility with RViz
        # RViz defaults to RELIABLE subscriptions for image topics
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        
        # QoS for high-bandwidth topics (pointcloud) - BEST_EFFORT to avoid drops
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        
        # Publishers matching RealSense topic names
        self.rgb_pub = self.create_publisher(
            Image, '/camera/color/image_raw', reliable_qos)
        self.depth_pub = self.create_publisher(
            Image, '/camera/depth/image_rect_raw', reliable_qos)
        self.rgb_info_pub = self.create_publisher(
            CameraInfo, '/camera/color/camera_info', reliable_qos)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, '/camera/depth/camera_info', reliable_qos)
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, '/camera/depth/color/points', sensor_qos)
        
        # Shared memory readers
        self.rgb_reader = MultiImageReader() if SHM_AVAILABLE else None
        self.depth_reader = DepthImageReader() if SHM_AVAILABLE else None
        
        # Timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self._publish_camera_data)
        
        # Stats
        self._connected_rgb = False
        self._connected_depth = False
        self._frame_count = 0
        self._last_log_time = time.time()
        
        self.get_logger().info(f"SimCameraROS2Bridge initialized")
        self.get_logger().info(f"  Frame ID: {frame_id}")
        self.get_logger().info(f"  Publishing to RealSense-compatible topics:")
        self.get_logger().info(f"    - /camera/color/image_raw")
        self.get_logger().info(f"    - /camera/depth/image_rect_raw")
        self.get_logger().info(f"    - /camera/depth/color/points")
    
    def _create_camera_info(self, stamp, is_depth: bool = False) -> CameraInfo:
        """Create CameraInfo message matching RealSense format"""
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id if not is_depth else f"{self.frame_id}_depth_optical_frame"
        msg.width = self.image_width
        msg.height = self.image_height
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion in simulation
        
        # Intrinsic matrix K
        msg.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0
        ]
        
        # Rectification matrix R (identity)
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Projection matrix P
        msg.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        return msg
    
    def _depth_to_pointcloud(self, depth_m: np.ndarray, rgb: np.ndarray, stamp) -> PointCloud2:
        """Convert depth image to PointCloud2 message
        
        Args:
            depth_m: Depth image in meters (float32)
            rgb: RGB image (uint8, HxWx3)
            stamp: ROS timestamp
            
        Returns:
            PointCloud2 message with XYZRGB points
        """
        height, width = depth_m.shape[:2]
        
        # Generate pixel coordinates
        u = np.arange(width, dtype=np.float32)
        v = np.arange(height, dtype=np.float32)
        u, v = np.meshgrid(u, v)
        
        # Back-project to 3D (camera frame: Z forward, X right, Y down)
        z = depth_m
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        
        # Filter valid points (0.1m to 10m range)
        valid = (z > 0.1) & (z < 10.0) & np.isfinite(z)
        
        x = x[valid].astype(np.float32)
        y = y[valid].astype(np.float32)
        z = z[valid].astype(np.float32)
        
        # Get colors
        if rgb is not None and rgb.shape[:2] == depth_m.shape[:2]:
            r = rgb[:, :, 0][valid].astype(np.uint8)
            g = rgb[:, :, 1][valid].astype(np.uint8)
            b = rgb[:, :, 2][valid].astype(np.uint8)
        else:
            r = g = b = np.full(len(x), 128, dtype=np.uint8)
        
        # Pack as structured array
        points = np.zeros(len(x), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32),
        ])
        points['x'] = x
        points['y'] = y
        points['z'] = z
        
        # Pack RGB as uint32 (RealSense format)
        rgb_packed = (r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | b.astype(np.uint32)
        points['rgb'] = rgb_packed
        
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = len(x)
        msg.is_dense = True
        msg.is_bigendian = False
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.data = points.tobytes()
        
        return msg
    
    def _publish_camera_data(self):
        """Read from shared memory and publish to ROS2"""
        if not SHM_AVAILABLE:
            return
        
        stamp = self.get_clock().now().to_msg()
        
        # Read RGB image
        rgb_img = None
        if self.rgb_reader:
            images = self.rgb_reader.read_images()
            if images and 'head' in images:
                rgb_img = images['head']
                if not self._connected_rgb:
                    self._connected_rgb = True
                    self.get_logger().info("Connected to RGB shared memory")
        
        # Read depth image
        depth_m = None
        if self.depth_reader:
            depth_m = self.depth_reader.read_depth()
            if depth_m is not None:
                if not self._connected_depth:
                    self._connected_depth = True
                    self.get_logger().info("Connected to depth shared memory")
        
        # Nothing to publish
        if rgb_img is None and depth_m is None:
            return
        
        self._frame_count += 1
        
        # Publish RGB image
        if rgb_img is not None:
            rgb_msg = Image()
            rgb_msg.header.stamp = stamp
            rgb_msg.header.frame_id = self.frame_id
            rgb_msg.height = rgb_img.shape[0]
            rgb_msg.width = rgb_img.shape[1]
            rgb_msg.encoding = 'bgr8'  # OpenCV format
            rgb_msg.is_bigendian = False
            rgb_msg.step = rgb_img.shape[1] * 3
            rgb_msg.data = rgb_img.tobytes()
            self.rgb_pub.publish(rgb_msg)
            
            # Publish RGB camera info
            rgb_info = self._create_camera_info(stamp, is_depth=False)
            self.rgb_info_pub.publish(rgb_info)
        
        # Publish depth image
        if depth_m is not None:
            # Convert to uint16 millimeters (RealSense format)
            depth_mm = (depth_m * 1000.0).astype(np.uint16)
            
            depth_msg = Image()
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = self.frame_id
            depth_msg.height = depth_mm.shape[0]
            depth_msg.width = depth_mm.shape[1]
            depth_msg.encoding = '16UC1'  # RealSense depth format
            depth_msg.is_bigendian = False
            depth_msg.step = depth_mm.shape[1] * 2
            depth_msg.data = depth_mm.tobytes()
            self.depth_pub.publish(depth_msg)
            
            # Publish depth camera info
            depth_info = self._create_camera_info(stamp, is_depth=True)
            self.depth_info_pub.publish(depth_info)
            
            # Publish point cloud
            pointcloud_msg = self._depth_to_pointcloud(depth_m, rgb_img, stamp)
            self.pointcloud_pub.publish(pointcloud_msg)
        
        # Log stats periodically
        now = time.time()
        if now - self._last_log_time > 10.0:
            fps = self._frame_count / (now - self._last_log_time)
            self.get_logger().info(f"Publishing at {fps:.1f} FPS (RGB: {self._connected_rgb}, Depth: {self._connected_depth})")
            self._frame_count = 0
            self._last_log_time = now
    
    def destroy_node(self):
        """Clean up resources"""
        if self.rgb_reader:
            self.rgb_reader.close()
        if self.depth_reader:
            self.depth_reader.close()
        super().destroy_node()


def main():
    """Main entry point"""
    if not ROS2_AVAILABLE:
        print("ERROR: ROS2 not available. Cannot run camera bridge.")
        return 1
    
    rclpy.init()
    
    # Get settings from environment
    frame_id = os.environ.get('CAMERA_FRAME', 'camera_link')
    publish_rate = float(os.environ.get('CAMERA_RATE', '30.0'))
    
    bridge = SimCameraROS2Bridge(
        frame_id=frame_id,
        publish_rate=publish_rate,
    )
    
    try:
        print("=" * 60)
        print("Simulation Camera ROS2 Bridge")
        print("=" * 60)
        print(f"Waiting for Isaac Lab simulation to start...")
        print(f"Camera data will be published to RealSense-compatible topics.")
        print("=" * 60)
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
