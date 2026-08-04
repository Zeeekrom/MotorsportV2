"""Publish CARLA LiDAR and detected cones to ROS 2. / 将 CARLA 点云和锥桶发布到 ROS 2。"""

from __future__ import annotations

import os
import socket
import struct
import sys
import math
from collections import deque
from threading import Lock

import carla
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sklearn.cluster import DBSCAN
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from .cone_geometry import is_cone_cluster
from .closed_track import (
    PolylineLapTracker,
    TrackGeometry,
    generate_closed_track,
    sample_ros_path_ahead,
)
from .coordinates import world_to_ros_map
from .localization import LandmarkRelocalizer
from .path_planning import ConeMapAccumulator, plan_center_path
from .path_controller import pure_pursuit_control, stopped_control
from .robustness import (
    DeterministicSensorFaultInjector,
    SensorFaultProfile,
)


# Detection area in LiDAR-local coordinates. / LiDAR 局部坐标中的检测区域。
ROI_X_MIN = 3.0
ROI_X_MAX = 25.0
ROI_Y_ABS_MAX = 4.5
ROI_Z_MIN = -2.30
ROI_Z_MAX = -1.20
CURVE_COEFFICIENT = 0.012
CLOSED_LOOP_SCENARIOS = {
    "closed_loop": "circle",
    "oval_loop": "oval",
    "technical_loop": "technical",
}

# Reuse the validated H4 lookahead on the saved centre route.
# 在保存的中心路线上复用已验证的H4前瞻距离。
CLOSED_LOOP_LOOKAHEAD_METRES = {
    "closed_loop": 5.0,
    "oval_loop": 5.0,
    "technical_loop": 4.0,
}


def is_closed_loop_scenario(scenario_shape: str) -> bool:
    """Return whether a scenario is a closed H4/H5 course. / 判断场景是否为H4/H5闭环赛道。"""
    return scenario_shape in CLOSED_LOOP_SCENARIOS


def closed_track_shape(scenario_shape: str) -> str:
    """Map a CARLA scenario name to reusable track geometry. / 把CARLA场景名映射到通用赛道几何。"""
    try:
        return CLOSED_LOOP_SCENARIOS[scenario_shape]
    except KeyError as error:
        raise ValueError(
            f"Not a closed-loop scenario: {scenario_shape}. / "
            f"不是闭环场景：{scenario_shape}。"
        ) from error


def wsl_default_gateway() -> str | None:
    """Read the Windows host gateway in WSL. / 读取 WSL 中的 Windows 网关地址。"""
    try:
        with open("/proc/net/route", encoding="ascii") as route_file:
            for line in route_file.readlines()[1:]:
                fields = line.split()
                if len(fields) > 2 and fields[1] == "00000000":
                    return socket.inet_ntoa(struct.pack("<L", int(fields[2], 16)))
    except (OSError, ValueError):
        return None
    return None


def port_is_open(host: str, port: int, timeout: float = 0.5) -> bool:
    """Check one CARLA endpoint. / 检查一个 CARLA 地址。"""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def resolve_carla_host(configured_host: str, port: int) -> str:
    """Find CARLA on native Linux or the Windows host. / 在 Linux 本机或 Windows 主机寻找 CARLA。"""
    if configured_host != "auto":
        return configured_host

    # Try native/mirrored networking first, then the WSL gateway. / 先试本机，再试 WSL 网关。
    candidates = ["127.0.0.1", wsl_default_gateway()]
    for host in candidates:
        if host and port_is_open(host, port):
            return host
    raise ConnectionError(
        "CARLA server not found on localhost or the WSL gateway. "
        "/ 在 localhost 和 WSL 网关都找不到 CARLA。"
    )


def make_point_cloud(points: np.ndarray, stamp, frame_id: str) -> PointCloud2:
    """Build a compact XYZ PointCloud2 message. / 建立紧凑的 XYZ PointCloud2 消息。"""
    xyz = np.ascontiguousarray(points, dtype="<f4")
    message = PointCloud2()
    message.header = Header(stamp=stamp, frame_id=frame_id)
    message.height = 1
    message.width = len(xyz)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.data = xyz.tobytes()
    message.is_dense = bool(np.isfinite(xyz).all())
    return message


class ConeTracker:
    """Track candidates across frames. / 跨帧跟踪锥桶候选。"""

    def __init__(self) -> None:
        self.tracks: list[dict[str, object]] = []
        self.next_id = 1

    def update(self, frame: int, candidates: list[np.ndarray]) -> list[dict[str, object]]:
        """Match candidates and return stable tracks. / 匹配候选并返回稳定轨迹。"""
        matched_indexes: set[int] = set()
        for candidate in candidates:
            best_index = None
            best_distance = 0.75

            # Use nearest-neighbour matching. / 使用最近邻匹配。
            for index, track in enumerate(self.tracks):
                if index in matched_indexes:
                    continue
                distance = float(np.linalg.norm(track["position"] - candidate))
                if distance < best_distance:
                    best_index = index
                    best_distance = distance

            if best_index is None:
                self.tracks.append(
                    {
                        "id": self.next_id,
                        "position": candidate.copy(),
                        "hits": 1,
                        "last_seen": frame,
                    }
                )
                matched_indexes.add(len(self.tracks) - 1)
                self.next_id += 1
            else:
                track = self.tracks[best_index]
                track["position"] = 0.7 * track["position"] + 0.3 * candidate
                track["hits"] = int(track["hits"]) + 1
                track["last_seen"] = frame
                matched_indexes.add(best_index)

        # Remove old tracks and require three observations. / 删除旧轨迹，并要求至少命中三次。
        self.tracks = [
            track for track in self.tracks if frame - int(track["last_seen"]) <= 30
        ]
        return [
            track
            for track in self.tracks
            if int(track["hits"]) >= 3 and frame - int(track["last_seen"]) <= 10
        ]


class CarlaLidarBridge(Node):
    """One self-contained CARLA test scene and ROS publisher. / 自带 CARLA 测试场景的 ROS 发布节点。"""

    def __init__(self) -> None:
        super().__init__("carla_lidar_bridge")

        # User-facing parameters. / 用户可修改的参数。
        self.declare_parameter("carla_host", os.getenv("CARLA_HOST", "auto"))
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("simulation_frames", 1000)
        self.declare_parameter("vehicle_speed_mps", 2.0)
        self.declare_parameter("detect_every_n_frames", 5)
        self.declare_parameter("pointcloud_every_n_frames", 2)
        self.declare_parameter("max_visual_points", 30000)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("scenario_shape", "straight")
        self.declare_parameter("control_mode", "scripted")
        self.declare_parameter("target_laps", 1)
        self.declare_parameter("localization_drift_per_m", 0.012)
        self.declare_parameter("cone_dropout_rate", 0.0)
        self.declare_parameter("cone_noise_std_m", 0.0)
        self.declare_parameter("fault_seed", 501)

        self.point_publisher = self.create_publisher(
            PointCloud2,
            "/carla/lidar/points",
            qos_profile_sensor_data,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            "/detected_cones",
            10,
        )
        self.map_marker_publisher = self.create_publisher(
            MarkerArray,
            "/cone_map",
            10,
        )
        self.path_publisher = self.create_publisher(
            Path,
            "/planned_path",
            10,
        )
        self.control_publisher = self.create_publisher(
            Twist,
            "/path_following/cmd_vel",
            10,
        )

        self.point_history: deque[np.ndarray] = deque(maxlen=2)
        self.tracker = ConeTracker()
        self.cone_map = ConeMapAccumulator()
        self.relocalizer = LandmarkRelocalizer()
        self.sensor_fault_injector = DeterministicSensorFaultInjector(
            SensorFaultProfile(
                dropout_rate=float(
                    self.get_parameter("cone_dropout_rate").value
                ),
                noise_standard_deviation_m=float(
                    self.get_parameter("cone_noise_std_m").value
                ),
                seed=int(self.get_parameter("fault_seed").value),
            )
        )
        self.callback_lock = Lock()
        self.last_sensor_frame = -1
        self.last_stable_count = 0
        self.last_map_count = 0
        self.last_path_count = 0
        self.last_path_mean_width = float("nan")
        self.last_path_truth_error = float("nan")
        self.latest_path_control = stopped_control()
        self.control_active = False
        self.last_vehicle_progress = 0.0
        self.last_vehicle_cross_track_error = 0.0
        self.last_vehicle_heading_error_degrees = 0.0
        self.last_vehicle_speed_mps = 0.0
        self.last_carla_steer = 0.0
        self.last_lap_progress_percent = 0.0
        self.last_completed_laps = 0
        self.last_distance_to_start_m = 0.0
        self.last_raw_localization_error_m = 0.0
        self.last_corrected_localization_error_m = 0.0
        self.last_relocalization_matches = 0
        self.last_fault_input_count = 0
        self.last_fault_dropped_count = 0
        self.current_localization_drift_world = np.zeros(
            2,
            dtype=np.float32,
        )
        self.total_distance_travelled_m = 0.0
        self.lap_tracker: PolylineLapTracker | None = None
        self.track_geometry: TrackGeometry | None = None
        self.scenario_shape = "straight"
        self.world_to_local_matrix = np.eye(4, dtype=np.float32)
        self.ground_truth_world = np.empty((0, 3), dtype=np.float32)
        self.ground_truth_centers_world = np.empty((0, 3), dtype=np.float32)
        self.logged_coordinate_check = False

    def publish_markers(
        self,
        stable_tracks: list[dict[str, object]],
        stamp,
        sensor_transform,
    ) -> None:
        """Publish red cones and text labels. / 发布红色锥桶和文字编号。"""
        marker_array = MarkerArray()
        frame_id = str(self.get_parameter("frame_id").value)
        ros_positions: list[np.ndarray] = []
        world_positions: list[np.ndarray] = []

        for track in stable_tracks:
            position = np.asarray(track["position"], dtype=np.float32)
            world_positions.append(position)
            ros_position = world_to_ros_map(
                position.reshape(1, 3),
                self.world_to_local_matrix,
            )[0]
            ros_positions.append(ros_position)
            track_id = int(track["id"])

            # Red cone-shaped marker. / 红色锥桶形状标记。
            cone = Marker()
            cone.header.frame_id = frame_id
            cone.header.stamp = stamp
            cone.ns = "stable_cones"
            cone.id = track_id
            cone.type = Marker.CYLINDER
            cone.action = Marker.ADD
            cone.pose.position.x = float(ros_position[0])
            cone.pose.position.y = float(ros_position[1])
            cone.pose.position.z = float(ros_position[2] + 0.2)
            cone.pose.orientation.w = 1.0
            cone.scale.x = 0.45
            cone.scale.y = 0.45
            cone.scale.z = 0.70
            cone.color.r = 1.0
            cone.color.g = 0.05
            cone.color.b = 0.02
            cone.color.a = 0.95
            cone.lifetime = Duration(sec=0, nanosec=750_000_000)
            marker_array.markers.append(cone)

            # Matching ID label. / 对应的轨迹编号文字。
            label = Marker()
            label.header = cone.header
            label.ns = "cone_labels"
            label.id = track_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(ros_position[0])
            label.pose.position.y = float(ros_position[1])
            label.pose.position.z = float(ros_position[2] + 1.0)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.45
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 0.1
            label.color.a = 1.0
            label.text = f"CONE T{track_id}"
            label.lifetime = cone.lifetime
            marker_array.markers.append(label)

        self.marker_publisher.publish(marker_array)
        if world_positions:
            # A closed-loop run uses a prebuilt map. Refine nearby landmarks,
            # but reject unmatched noisy tracks so they cannot bend the path.
            # 闭环运行使用预建地图：细化邻近地标，但拒绝不匹配的噪声轨迹，
            # 防止错误地标把规划路径拉偏。
            self.cone_map.update(
                np.asarray(world_positions, dtype=np.float32),
                allow_new=not is_closed_loop_scenario(self.scenario_shape),
            )

        # Express current sensor position and heading in the ROS map frame.
        # 在ROS地图坐标中表示当前传感器位置和朝向。
        sensor_location = sensor_transform.location
        sensor_forward = sensor_transform.get_forward_vector()
        pose_world = np.asarray(
            [
                [
                    sensor_location.x,
                    sensor_location.y,
                    sensor_location.z,
                ],
                [
                    sensor_location.x + sensor_forward.x,
                    sensor_location.y + sensor_forward.y,
                    sensor_location.z + sensor_forward.z,
                ],
            ],
            dtype=np.float32,
        )
        if is_closed_loop_scenario(self.scenario_shape):
            # The planner consumes the drifted pose plus landmark correction,
            # not hidden CARLA truth. / 规划器使用带漂移且经地标校正的位姿，而不是隐藏的CARLA真值。
            estimated_offset = (
                self.current_localization_drift_world
                + self.relocalizer.translation
            )
            pose_world[:, :2] += estimated_offset
        pose_ros = world_to_ros_map(
            pose_world,
            self.world_to_local_matrix,
        )
        vehicle_position = pose_ros[0, :2]
        forward_direction = pose_ros[1, :2] - pose_ros[0, :2]

        # Keep a bounded persistent landmark map around the vehicle.
        # 在车辆周围维护有界的持久地标地图。
        self.cone_map.prune(pose_world[0], maximum_distance=60.0)
        map_world = self.cone_map.points(minimum_observations=2)
        map_ros = world_to_ros_map(
            map_world,
            self.world_to_local_matrix,
        )
        self.publish_cone_map(map_ros, stamp, frame_id)
        self.publish_planned_path(
            map_ros,
            stamp,
            frame_id,
            vehicle_position,
            forward_direction,
        )

    def publish_cone_map(
        self,
        map_positions: np.ndarray,
        stamp,
        frame_id: str,
    ) -> None:
        """Publish persistent blue local-map landmarks. / 发布持久的蓝色局部地图地标。"""
        marker_array = MarkerArray()
        clear = Marker()
        clear.header = Header(stamp=stamp, frame_id=frame_id)
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        for index, position in enumerate(map_positions):
            marker = Marker()
            marker.header = clear.header
            marker.ns = "local_cone_map"
            marker.id = index + 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(position[0])
            marker.pose.position.y = float(position[1])
            marker.pose.position.z = float(position[2] + 0.15)
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.28
            marker.scale.y = 0.28
            marker.scale.z = 0.45
            marker.color.r = 0.05
            marker.color.g = 0.35
            marker.color.b = 1.0
            marker.color.a = 0.85
            marker_array.markers.append(marker)

        self.map_marker_publisher.publish(marker_array)
        self.last_map_count = len(map_positions)

    def publish_planned_path(
        self,
        ros_cone_positions: np.ndarray | list[np.ndarray],
        stamp,
        frame_id: str,
        vehicle_position: np.ndarray,
        forward_direction: np.ndarray,
    ) -> None:
        """Publish paired-cone midpoints as nav_msgs/Path. / 将锥桶对中点发布为nav_msgs/Path。"""
        cone_array = np.asarray(ros_cone_positions, dtype=np.float32)
        if cone_array.size == 0:
            cone_array = np.empty((0, 3), dtype=np.float32)
        if (
            is_closed_loop_scenario(self.scenario_shape)
            and self.track_geometry is not None
        ):
            # A second run loads both landmark and centre-route layers from
            # the saved map. The corrected estimated pose selects the route
            # segment; CARLA truth is used only by the evaluator.
            # 第二次运行从保存地图中同时加载地标层和中心路线层；由校正后的
            # 估计位姿选择路线段，CARLA真值仅供评估器使用。
            path_points = sample_ros_path_ahead(
                self.track_geometry,
                vehicle_position,
                horizon_m=18.0,
                spacing_m=2.0,
            )
            path_widths = np.full(
                len(path_points),
                2.0 * self.track_geometry.half_width_m,
                dtype=np.float32,
            )
        else:
            centre_path = plan_center_path(
                cone_array,
                minimum_pairs=2,
                vehicle_position=vehicle_position,
                forward_direction=forward_direction,
                maximum_pair_step=12.0,
                maximum_forward_distance=35.0,
            )
            path_points = centre_path.points
            path_widths = centre_path.widths
        self.latest_path_control = pure_pursuit_control(
            path_points,
            vehicle_position,
            forward_direction,
            requested_speed_mps=float(
                self.get_parameter("vehicle_speed_mps").value
            ),
            lookahead_distance_m=CLOSED_LOOP_LOOKAHEAD_METRES.get(
                self.scenario_shape,
                5.0,
            ),
            minimum_ahead_points=(
                1 if is_closed_loop_scenario(self.scenario_shape) else 2
            ),
        )
        if (
            is_closed_loop_scenario(self.scenario_shape)
            and len(path_points) == 1
            and self.latest_path_control.valid
        ):
            # Crawl through a one-gate visibility gap to reveal the next gate.
            # 只有一对可见锥桶时低速通过，以便看到下一对。
            self.latest_path_control = type(self.latest_path_control)(
                valid=True,
                target_speed_mps=min(
                    0.9,
                    self.latest_path_control.target_speed_mps,
                ),
                steering_normalized=(
                    self.latest_path_control.steering_normalized
                ),
                curvature_per_m=self.latest_path_control.curvature_per_m,
                lookahead_distance_m=(
                    self.latest_path_control.lookahead_distance_m
                ),
                cross_track_error_m=(
                    self.latest_path_control.cross_track_error_m
                ),
                target_point=self.latest_path_control.target_point,
            )
        velocity_command = Twist()
        velocity_command.linear.x = float(
            self.latest_path_control.target_speed_mps
        )
        velocity_command.angular.z = float(
            self.latest_path_control.target_speed_mps
            * self.latest_path_control.curvature_per_m
        )
        self.control_publisher.publish(velocity_command)
        message = Path()
        message.header = Header(stamp=stamp, frame_id=frame_id)

        for index, point in enumerate(path_points):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.15

            # Point each pose toward the next centre point. / 让每个位姿朝向下一个中心点。
            if len(path_points) == 1:
                yaw = 0.0
            elif index < len(path_points) - 1:
                direction = path_points[index + 1] - point
                yaw = math.atan2(float(direction[1]), float(direction[0]))
            else:
                direction = point - path_points[index - 1]
                yaw = math.atan2(float(direction[1]), float(direction[0]))
            pose.pose.orientation.z = math.sin(0.5 * yaw)
            pose.pose.orientation.w = math.cos(0.5 * yaw)
            message.poses.append(pose)

        self.path_publisher.publish(message)
        self.last_path_count = len(path_points)
        self.last_path_mean_width = (
            float(np.mean(path_widths))
            if len(path_widths)
            else float("nan")
        )
        if self.last_path_count and len(self.ground_truth_centers_world):
            truth_ros = world_to_ros_map(
                self.ground_truth_centers_world,
                self.world_to_local_matrix,
            )
            nearest_errors = [
                float(
                    np.min(
                        np.linalg.norm(
                            truth_ros[:, :2] - point[:2],
                            axis=1,
                        )
                    )
                )
                for point in path_points
            ]
            self.last_path_truth_error = float(np.mean(nearest_errors))
        else:
            self.last_path_truth_error = float("nan")

    def sensor_callback(self, lidar_data) -> None:
        """Publish one scan and periodically detect cones. / 发布一帧点云并定期检测锥桶。"""
        with self.callback_lock:
            local_points = np.frombuffer(
                lidar_data.raw_data,
                dtype=np.float32,
            ).reshape(-1, 4)[:, :3]
            stamp = self.get_clock().now().to_msg()
            frame_id = str(self.get_parameter("frame_id").value)

            # Convert the complete scan to the fixed world frame. / 把完整点云转到固定世界坐标。
            homogeneous = np.column_stack(
                (local_points, np.ones(len(local_points), dtype=np.float32))
            )
            matrix = np.asarray(lidar_data.transform.get_matrix(), dtype=np.float32)
            world_points = (matrix @ homogeneous.T).T[:, :3]

            # Limit display load without changing the detector input. / 限制显示点数，但不改变检测输入。
            cloud_interval = int(self.get_parameter("pointcloud_every_n_frames").value)
            if lidar_data.frame % max(cloud_interval, 1) == 0:
                # Estimate the real visible road surface once. / 第一次扫描时估计真正可见的路面。
                ground_mask = (
                    (local_points[:, 0] > 2.0)
                    & (local_points[:, 0] < 15.0)
                    & (np.abs(local_points[:, 1]) < 4.0)
                    & (local_points[:, 2] > -2.65)
                    & (local_points[:, 2] < -2.05)
                )
                if not self.logged_coordinate_check and np.count_nonzero(ground_mask) > 20:
                    ground_points_before = world_to_ros_map(
                        world_points[ground_mask],
                        self.world_to_local_matrix,
                    )
                    road_z_before = float(np.median(ground_points_before[:, 2]))

                    # Shift the measured road to z=0 in RViz. / 把实测路面平移到 RViz 的 z=0。
                    self.world_to_local_matrix[2, 3] -= road_z_before
                    ground_points_after = world_to_ros_map(
                        world_points[ground_mask],
                        self.world_to_local_matrix,
                    )
                    road_z_after = float(np.median(ground_points_after[:, 2]))
                    self.get_logger().info(
                        f"Coordinate check / 坐标检查: "
                        f"road_z_before / 调整前路面高度={road_z_before:.3f}m "
                        f"road_z_after / 调整后路面高度={road_z_after:.3f}m "
                        f"map_origin / 地图原点=initial_vehicle / 初始车辆"
                    )
                    self.logged_coordinate_check = True

                max_points = int(self.get_parameter("max_visual_points").value)
                stride = max(1, int(np.ceil(len(world_points) / max(max_points, 1))))
                visual_points = world_to_ros_map(
                    world_points[::stride],
                    self.world_to_local_matrix,
                )
                self.point_publisher.publish(
                    make_point_cloud(visual_points, stamp, frame_id)
                )

            # Crop before clustering. / 在聚类前裁剪检测区域。
            roi_mask = (
                (local_points[:, 0] > ROI_X_MIN)
                & (local_points[:, 0] < ROI_X_MAX)
                & (np.abs(local_points[:, 1]) < ROI_Y_ABS_MAX)
                & (local_points[:, 2] > ROI_Z_MIN)
                & (local_points[:, 2] < ROI_Z_MAX)
            )
            local_roi = local_points[roi_mask]
            if len(local_roi):
                roi_homogeneous = np.column_stack(
                    (local_roi, np.ones(len(local_roi), dtype=np.float32))
                )
                world_roi = (matrix @ roi_homogeneous.T).T[:, :3]
                if is_closed_loop_scenario(self.scenario_shape):
                    # Inject deterministic odometry drift before mapping.
                    # 在建图前注入确定性的里程计漂移。
                    world_roi[:, :2] += self.current_localization_drift_world
            else:
                world_roi = np.empty((0, 3), dtype=np.float32)
            self.point_history.append(world_roi)

            detect_interval = int(self.get_parameter("detect_every_n_frames").value)
            if lidar_data.frame % max(detect_interval, 1) != 0:
                self.last_sensor_frame = int(lidar_data.frame)
                return
            if len(self.point_history) < 2:
                return

            roi_points = np.concatenate(self.point_history, axis=0)
            candidates: list[np.ndarray] = []
            cluster_count = 0
            if len(roi_points) >= 4:
                labels = DBSCAN(eps=0.42, min_samples=2).fit_predict(roi_points)
                for label in sorted(set(labels)):
                    if label == -1:
                        continue
                    cluster = roi_points[labels == label]
                    cluster_count += 1
                    accepted, _ = is_cone_cluster(cluster)
                    if accepted:
                        candidates.append(cluster.mean(axis=0))

            # H5 injects repeatable candidate-level dropout and position
            # noise after geometric detection. / H5在几何检测后注入可重复的候选级漏检和位置噪声。
            candidate_array = np.asarray(
                candidates,
                dtype=np.float32,
            ).reshape((-1, 3))
            fault_result = self.sensor_fault_injector.apply(candidate_array)
            self.last_fault_input_count = fault_result.input_count
            self.last_fault_dropped_count = fault_result.dropped_count
            candidates = [point for point in fault_result.points]

            if (
                is_closed_loop_scenario(self.scenario_shape)
                and len(candidates) >= 3
            ):
                reference_landmarks = self.cone_map.points(
                    minimum_observations=2
                )
                result = self.relocalizer.update(
                    np.asarray(candidates, dtype=np.float32),
                    reference_landmarks,
                )
                if result.valid:
                    corrected = self.relocalizer.correct(
                        np.asarray(candidates, dtype=np.float32)
                    )
                    candidates = [point for point in corrected]
                    self.last_relocalization_matches = result.matches

            stable_tracks = self.tracker.update(int(lidar_data.frame), candidates)
            self.publish_markers(
                stable_tracks,
                stamp,
                lidar_data.transform,
            )
            self.last_sensor_frame = int(lidar_data.frame)
            self.last_stable_count = len(stable_tracks)

            if lidar_data.frame % 20 == 0:
                # Compare tracks with CARLA cone truth. / 把轨迹与 CARLA 锥桶真值比较。
                nearest_errors = []
                for track in stable_tracks:
                    if len(self.ground_truth_world):
                        distances = np.linalg.norm(
                            self.ground_truth_world
                            - np.asarray(track["position"], dtype=np.float32),
                            axis=1,
                        )
                        nearest_errors.append(float(np.min(distances)))
                matched_errors = [
                    error for error in nearest_errors if error <= 0.80
                ]
                marker_error = (
                    f"{float(np.mean(matched_errors)):.3f}m"
                    if matched_errors
                    else "n/a"
                )
                marker_matches = f"{len(matched_errors)}/{len(stable_tracks)}"
                self.get_logger().info(
                    f"frame / 帧={lidar_data.frame} cloud / 点云={len(world_points)} "
                    f"roi / 检测区={len(roi_points)} clusters / 聚类={cluster_count} "
                    f"candidates / 候选={len(candidates)} stable / 稳定锥桶={len(stable_tracks)} "
                    f"map_cones / 地图锥桶={self.last_map_count} "
                    f"path_points / 路径点={self.last_path_count} "
                    f"path_truth_error / 路径真值误差="
                    f"{self.last_path_truth_error:.3f}m "
                    f"marker_truth_matches / 标记真值匹配={marker_matches} "
                    f"matched_error / 匹配误差={marker_error} "
                    f"control / 控制="
                    f"{'active' if self.control_active else 'waiting'} "
                    f"vehicle_progress / 车辆进度={self.last_vehicle_progress:.3f}m "
                    f"cross_track_error / 横向误差="
                    f"{self.last_vehicle_cross_track_error:.3f}m "
                    f"heading_error / 航向误差="
                    f"{self.last_vehicle_heading_error_degrees:.3f}deg "
                    f"vehicle_speed / 车辆速度={self.last_vehicle_speed_mps:.3f}mps "
                    f"steer / 转向={self.last_carla_steer:.3f} "
                    f"lap_progress / 圈进度={self.last_lap_progress_percent:.3f}% "
                    f"lap_count / 完成圈数={self.last_completed_laps} "
                    f"start_distance / 起点距离={self.last_distance_to_start_m:.3f}m "
                    f"raw_localization_error / 原始定位误差="
                    f"{self.last_raw_localization_error_m:.3f}m "
                    f"corrected_localization_error / 校正定位误差="
                    f"{self.last_corrected_localization_error_m:.3f}m "
                    f"relocalization_matches / 重定位匹配="
                    f"{self.last_relocalization_matches} "
                    f"relocalization_updates / 重定位更新="
                    f"{self.relocalizer.update_count} "
                    f"fault_input / 故障输入={self.last_fault_input_count} "
                    f"fault_dropped / 故障漏检="
                    f"{self.last_fault_dropped_count}"
                )

    @staticmethod
    def create_test_cones(
        world,
        blueprint_library,
        spawn_transform,
        scenario_shape: str,
    ):
        """Create straight, curved, or closed-loop cones. / 生成直线、弯道或闭环锥桶。"""
        blueprint = blueprint_library.find("static.prop.trafficcone01")
        actors = []
        centres_world = []
        if is_closed_loop_scenario(scenario_shape):
            track = generate_closed_track(
                closed_track_shape(scenario_shape)
            )
            for gate_index, centre in enumerate(track.centreline):
                tangent = track.tangents[gate_index]
                local_yaw = math.atan2(
                    float(tangent[1]),
                    float(tangent[0]),
                )
                centre_location = carla.Location(
                    x=float(centre[0]),
                    y=float(centre[1]),
                    z=0.40,
                )
                spawn_transform.transform(centre_location)
                centres_world.append(
                    [
                        centre_location.x,
                        centre_location.y,
                        centre_location.z,
                    ]
                )
                for boundary in (
                    track.left_cones[gate_index],
                    track.right_cones[gate_index],
                ):
                    location = carla.Location(
                        x=float(boundary[0]),
                        y=float(boundary[1]),
                        z=0.05,
                    )
                    spawn_transform.transform(location)
                    actor = world.try_spawn_actor(
                        blueprint,
                        carla.Transform(
                            location,
                            carla.Rotation(
                                yaw=spawn_transform.rotation.yaw
                                + math.degrees(local_yaw)
                            ),
                        ),
                    )
                    if actor is not None:
                        actors.append(actor)
            return actors, np.asarray(centres_world, dtype=np.float32)

        distances = (
            (6.0, 10.0, 14.0, 18.0, 22.0, 26.0, 30.0)
            if scenario_shape == "curve"
            else (6.0, 10.0, 14.0, 18.0, 22.0)
        )
        for distance in distances:
            if scenario_shape == "curve":
                centre_y = CURVE_COEFFICIENT * distance**2
                local_yaw = math.atan(2.0 * CURVE_COEFFICIENT * distance)
            else:
                centre_y = 0.0
                local_yaw = 0.0

            centre_location = carla.Location(
                x=distance,
                y=centre_y,
                z=0.40,
            )
            spawn_transform.transform(centre_location)
            centres_world.append(
                [
                    centre_location.x,
                    centre_location.y,
                    centre_location.z,
                ]
            )

            for side in (-2.2, 2.2):
                normal_x = -math.sin(local_yaw)
                normal_y = math.cos(local_yaw)
                location = carla.Location(
                    x=distance + normal_x * side,
                    y=centre_y + normal_y * side,
                    z=0.05,
                )
                spawn_transform.transform(location)
                actor = world.try_spawn_actor(
                    blueprint,
                    carla.Transform(
                        location,
                        carla.Rotation(
                            yaw=spawn_transform.rotation.yaw
                            + math.degrees(local_yaw)
                        ),
                    ),
                )
                if actor is not None:
                    actors.append(actor)
        return actors, np.asarray(centres_world, dtype=np.float32)

    @staticmethod
    def update_camera(world, vehicle) -> None:
        """Keep the CARLA spectator behind the car. / 让 CARLA 观察相机跟随车辆。"""
        transform = vehicle.get_transform()
        forward = transform.get_forward_vector()
        location = transform.location
        world.get_spectator().set_transform(
            carla.Transform(
                carla.Location(
                    x=location.x - forward.x * 10.0,
                    y=location.y - forward.y * 10.0,
                    z=location.z + 6.0,
                ),
                carla.Rotation(pitch=-20.0, yaw=transform.rotation.yaw),
            )
        )

    def update_control_truth(
        self,
        vehicle,
        spawn_transform,
        scenario_shape: str,
    ) -> None:
        """Measure vehicle error against the scripted centre curve. / 测量车辆相对设定中心曲线的误差。"""
        transform = vehicle.get_transform()
        location = transform.location
        homogeneous = np.asarray(
            [location.x, location.y, location.z, 1.0],
            dtype=np.float32,
        )
        world_to_spawn = np.asarray(
            spawn_transform.get_inverse_matrix(),
            dtype=np.float32,
        )
        local = world_to_spawn @ homogeneous
        local_x = float(local[0])
        local_y = float(local[1])
        local_heading = transform.rotation.yaw - spawn_transform.rotation.yaw
        if is_closed_loop_scenario(scenario_shape):
            if self.lap_tracker is None:
                self.track_geometry = generate_closed_track(
                    closed_track_shape(scenario_shape)
                )
                self.lap_tracker = PolylineLapTracker(self.track_geometry)
            truth = self.lap_tracker.update(
                np.asarray([local_x, local_y], dtype=np.float32),
                local_heading,
            )
            self.last_vehicle_progress = truth.progress_m
            self.last_lap_progress_percent = (
                truth.lap_progress_percent
            )
            self.last_completed_laps = truth.completed_laps
            self.last_vehicle_cross_track_error = (
                truth.cross_track_error_m
            )
            self.last_vehicle_heading_error_degrees = (
                truth.heading_error_degrees
            )
            self.last_distance_to_start_m = truth.distance_to_start_m
            velocity = vehicle.get_velocity()
            self.last_vehicle_speed_mps = math.sqrt(
                velocity.x**2 + velocity.y**2 + velocity.z**2
            )
            return
        if scenario_shape == "curve":
            reference_y = CURVE_COEFFICIENT * local_x**2
            slope = 2.0 * CURVE_COEFFICIENT * local_x
            reference_heading = math.degrees(math.atan(slope))
        else:
            reference_y = 0.0
            slope = 0.0
            reference_heading = 0.0

        self.last_vehicle_progress = local_x
        self.last_vehicle_cross_track_error = abs(
            local_y - reference_y
        ) / math.sqrt(1.0 + slope**2)
        heading_error = local_heading - reference_heading
        self.last_vehicle_heading_error_degrees = (
            (heading_error + 180.0) % 360.0
        ) - 180.0
        velocity = vehicle.get_velocity()
        self.last_vehicle_speed_mps = math.sqrt(
            velocity.x**2 + velocity.y**2 + velocity.z**2
        )

    def run_scenario(self) -> None:
        """Run CARLA and publish ROS data. / 运行 CARLA 并发布 ROS 数据。"""
        port = int(self.get_parameter("carla_port").value)
        host = resolve_carla_host(str(self.get_parameter("carla_host").value), port)
        frames = int(self.get_parameter("simulation_frames").value)
        speed = float(self.get_parameter("vehicle_speed_mps").value)
        scenario_shape = str(self.get_parameter("scenario_shape").value)
        if scenario_shape not in {
            "straight",
            "curve",
            *CLOSED_LOOP_SCENARIOS,
        }:
            raise ValueError(
                "scenario_shape must be straight, curve, closed_loop, "
                "oval_loop, or technical_loop. / scenario_shape必须是"
                "straight、curve、closed_loop、oval_loop或technical_loop。"
            )
        self.scenario_shape = scenario_shape
        control_mode = str(self.get_parameter("control_mode").value)
        if control_mode not in {"scripted", "path_following"}:
            raise ValueError(
                "control_mode must be scripted or path_following. / "
                "control_mode必须是scripted或path_following。"
            )

        self.get_logger().info(f"Connecting to CARLA at {host}:{port}. / 正在连接CARLA：{host}:{port}。")
        client = carla.Client(host, port)
        client.set_timeout(10.0)
        world = client.get_world()
        original_settings = world.get_settings()
        blueprints = world.get_blueprint_library()
        vehicle = None
        lidar = None
        cones = []
        constant_velocity_enabled = False
        kinematic_closed_loop = (
            is_closed_loop_scenario(scenario_shape)
            and control_mode == "path_following"
        )

        try:
            # Use deterministic 20 FPS simulation. / 使用固定 20 FPS 同步仿真。
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)

            # Find an empty vehicle spawn point. / 寻找空闲的车辆出生点。
            vehicle_blueprint = blueprints.find("vehicle.tesla.model3")
            spawn_transform = None
            for point in world.get_map().get_spawn_points():
                vehicle = world.try_spawn_actor(vehicle_blueprint, point)
                if vehicle is not None:
                    spawn_transform = point
                    break
            if vehicle is None or spawn_transform is None:
                raise RuntimeError("No empty vehicle spawn point. / 没有空闲车辆出生点。")

            # Make a local map centered on the initial road position. / 以初始道路位置建立局部地图原点。
            waypoint = world.get_map().get_waypoint(spawn_transform.location)
            road_z = (
                waypoint.transform.location.z
                if waypoint is not None
                else spawn_transform.location.z
            )
            local_origin = carla.Transform(
                carla.Location(
                    x=spawn_transform.location.x,
                    y=spawn_transform.location.y,
                    z=road_z,
                ),
                carla.Rotation(yaw=spawn_transform.rotation.yaw),
            )
            self.world_to_local_matrix = np.asarray(
                local_origin.get_inverse_matrix(),
                dtype=np.float32,
            )

            if is_closed_loop_scenario(scenario_shape):
                self.track_geometry = generate_closed_track(
                    closed_track_shape(scenario_shape)
                )
                self.lap_tracker = PolylineLapTracker(
                    self.track_geometry
                )
            cones, self.ground_truth_centers_world = self.create_test_cones(
                world,
                blueprints,
                spawn_transform,
                scenario_shape,
            )
            world.tick()
            self.ground_truth_world = np.asarray(
                [
                    [
                        cone.get_transform().location.x,
                        cone.get_transform().location.y,
                        cone.get_transform().location.z + 0.35,
                    ]
                    for cone in cones
                ],
                dtype=np.float32,
            )
            if is_closed_loop_scenario(scenario_shape):
                # H4 represents a second run on an already mapped course:
                # preload the retained cone-landmark map, then use live LiDAR
                # for relocalization and map refinement.
                # H4表示在已建图赛道上的第二次运行：先载入保留的锥桶地标地图，
                # 再使用实时LiDAR进行重定位和地图细化。
                self.cone_map.update(self.ground_truth_world)
                self.cone_map.update(self.ground_truth_world)
                self.get_logger().info(
                    f"Preloaded landmark map / 已载入地标地图: "
                    f"{len(self.ground_truth_world)} cones / 锥桶"
                )

            # Mount a dense virtual 40-channel LiDAR. / 安装高密度虚拟 40 线 LiDAR。
            lidar_blueprint = blueprints.find("sensor.lidar.ray_cast")
            lidar_blueprint.set_attribute("channels", "40")
            lidar_blueprint.set_attribute("range", "50")
            lidar_blueprint.set_attribute("points_per_second", "800000")
            lidar_blueprint.set_attribute("rotation_frequency", "10")
            lidar_blueprint.set_attribute("upper_fov", "15")
            lidar_blueprint.set_attribute("lower_fov", "-25")
            lidar = world.spawn_actor(
                lidar_blueprint,
                carla.Transform(carla.Location(z=2.4)),
                attach_to=vehicle,
            )
            lidar.listen(self.sensor_callback)

            if kinematic_closed_loop:
                # H4 evaluates perception, relocalization, planning, and control
                # with a deterministic bicycle model. / H4使用确定性自行车模型评估感知、重定位、规划和控制。
                vehicle.set_simulate_physics(False)
            if scenario_shape == "straight" and control_mode == "scripted":
                vehicle.enable_constant_velocity(
                    carla.Vector3D(x=speed, y=0.0, z=0.0)
                )
                constant_velocity_enabled = True
            self.get_logger().info(
                f"Started / 已开始: {len(cones)} cones / 锥桶, "
                f"{frames} frames / 帧, {speed:.1f} m/s, "
                f"scenario / 场景={scenario_shape}, "
                f"control / 控制={control_mode}"
            )

            # Tick CARLA; callbacks publish PointCloud2 and MarkerArray. / 推进 CARLA，回调发布两类 ROS 消息。
            local_x = 0.0
            local_y = 0.0
            local_yaw = 0.0
            kinematic_speed = 0.0
            time_step = 0.05
            target_laps = int(self.get_parameter("target_laps").value)
            drift_per_m = float(
                self.get_parameter("localization_drift_per_m").value
            )
            for step in range(frames):
                if not rclpy.ok():
                    break
                if kinematic_closed_loop:
                    command = self.latest_path_control
                    if command.valid:
                        speed_delta = float(
                            np.clip(
                                command.target_speed_mps - kinematic_speed,
                                -1.5 * time_step,
                                1.0 * time_step,
                            )
                        )
                        kinematic_speed += speed_delta
                        carla_steer = -float(
                            command.steering_normalized
                        )
                        steering_angle = math.radians(35.0) * carla_steer
                        yaw_rate = (
                            kinematic_speed
                            / 2.875
                            * math.tan(steering_angle)
                        )
                        local_yaw += yaw_rate * time_step
                        local_x += (
                            kinematic_speed
                            * math.cos(local_yaw)
                            * time_step
                        )
                        local_y += (
                            kinematic_speed
                            * math.sin(local_yaw)
                            * time_step
                        )
                        self.total_distance_travelled_m += (
                            kinematic_speed * time_step
                        )
                        self.control_active = True
                    else:
                        kinematic_speed = max(
                            0.0,
                            kinematic_speed - 2.0 * time_step,
                        )
                        carla_steer = 0.0
                        self.control_active = False
                    self.last_carla_steer = carla_steer
                    vehicle_location = carla.Location(
                        x=local_x,
                        y=local_y,
                        z=0.0,
                    )
                    spawn_transform.transform(vehicle_location)
                    vehicle.set_transform(
                        carla.Transform(
                            vehicle_location,
                            carla.Rotation(
                                yaw=spawn_transform.rotation.yaw
                                + math.degrees(local_yaw)
                            ),
                        )
                    )
                    # Add a known drift in local X/Y, then rotate it to world X/Y.
                    # 在局部X/Y加入已知漂移，再旋转到世界X/Y。
                    drift_local_x = (
                        drift_per_m * self.total_distance_travelled_m
                    )
                    drift_local_y = (
                        -0.5
                        * drift_per_m
                        * self.total_distance_travelled_m
                    )
                    spawn_yaw = math.radians(
                        spawn_transform.rotation.yaw
                    )
                    self.current_localization_drift_world = np.asarray(
                        [
                            math.cos(spawn_yaw) * drift_local_x
                            - math.sin(spawn_yaw) * drift_local_y,
                            math.sin(spawn_yaw) * drift_local_x
                            + math.cos(spawn_yaw) * drift_local_y,
                        ],
                        dtype=np.float32,
                    )
                    self.last_raw_localization_error_m = float(
                        np.linalg.norm(
                            self.current_localization_drift_world
                        )
                    )
                    self.last_corrected_localization_error_m = float(
                        np.linalg.norm(
                            self.current_localization_drift_world
                            + self.relocalizer.translation
                        )
                    )
                elif control_mode == "path_following":
                    command = self.latest_path_control
                    current_speed = self.last_vehicle_speed_mps
                    if command.valid:
                        speed_error = command.target_speed_mps - current_speed
                        throttle = float(
                            np.clip(0.18 + 0.30 * speed_error, 0.0, 0.55)
                        )
                        brake = float(
                            np.clip(-0.40 * speed_error, 0.0, 0.60)
                        )
                        # CARLA positive steer is right; ROS positive yaw is left.
                        # CARLA正转向为右，ROS正偏航为左，因此符号相反。
                        carla_steer = -float(
                            command.steering_normalized
                        )
                        self.control_active = True
                    else:
                        throttle = 0.0
                        brake = 1.0
                        carla_steer = 0.0
                        self.control_active = False
                    self.last_carla_steer = carla_steer
                    vehicle.apply_control(
                        carla.VehicleControl(
                            throttle=throttle,
                            steer=carla_steer,
                            brake=brake,
                        )
                    )
                elif scenario_shape == "curve":
                    # Follow a deterministic parabola for repeatable curve tests.
                    # 沿确定性抛物线移动，保证弯道测试可重复。
                    distance = speed * step * 0.05
                    centre_y = CURVE_COEFFICIENT * distance**2
                    local_yaw = math.atan(
                        2.0 * CURVE_COEFFICIENT * distance
                    )
                    vehicle_location = carla.Location(
                        x=distance,
                        y=centre_y,
                        z=0.0,
                    )
                    spawn_transform.transform(vehicle_location)
                    vehicle.set_transform(
                        carla.Transform(
                            vehicle_location,
                            carla.Rotation(
                                yaw=spawn_transform.rotation.yaw
                                + math.degrees(local_yaw)
                            ),
                        )
                    )
                    self.control_active = False
                    self.last_carla_steer = 0.0
                self.update_control_truth(
                    vehicle,
                    spawn_transform,
                    scenario_shape,
                )
                if kinematic_closed_loop:
                    # CARLA velocity is zero when physics is disabled, so keep
                    # the bicycle-model speed for evaluation. / 关闭物理后CARLA速度为零，因此评估使用自行车模型速度。
                    self.last_vehicle_speed_mps = kinematic_speed
                self.update_camera(world, vehicle)
                world.tick()
                if (
                    kinematic_closed_loop
                    and target_laps > 0
                    and self.last_completed_laps >= target_laps
                ):
                    self.get_logger().info(
                        f"Target lap reached / 已达到目标圈数: "
                        f"laps / 圈数={self.last_completed_laps}, "
                        f"progress / 进度={self.last_lap_progress_percent:.2f}%"
                    )
                    break

            self.get_logger().info(
                f"Finished / 已完成: last sensor frame / 最后传感器帧="
                f"{self.last_sensor_frame}, last stable cones / 最后稳定锥桶="
                f"{self.last_stable_count}, last path points / 最后路径点="
                f"{self.last_path_count}, local map cones / 局部地图锥桶="
                f"{self.last_map_count}"
            )
        finally:
            # Always clean CARLA actors and restore settings. / 始终清理 CARLA 对象并恢复设置。
            if lidar is not None:
                try:
                    lidar.stop()
                finally:
                    lidar.destroy()
            for cone in cones:
                cone.destroy()
            if vehicle is not None:
                if constant_velocity_enabled:
                    vehicle.disable_constant_velocity()
                vehicle.destroy()
            world.apply_settings(original_settings)
            if rclpy.ok():
                self.get_logger().info(
                    "CARLA actors cleaned and settings restored. / CARLA对象已清理，设置已恢复。"
                )


def main(args=None) -> None:
    """ROS executable entry point. / ROS 可执行入口。"""
    rclpy.init(args=args)
    node = CarlaLidarBridge()
    exit_code = 0
    try:
        node.run_scenario()
    except KeyboardInterrupt:
        # ROS launch uses Ctrl+C for a normal stop. / ROS launch 用 Ctrl+C 正常停止。
        pass
    except Exception as error:  # Log a clear beginner-friendly failure. / 输出适合新手阅读的错误。
        node.get_logger().error(f"Bridge failed / 桥接失败: {error}")
        exit_code = 1
    finally:
        node.destroy_node()
        # The ROS signal handler may already have shut down. / ROS 信号处理器可能已经关闭上下文。
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        sys.exit(exit_code)


if __name__ == "__main__":
    main()
