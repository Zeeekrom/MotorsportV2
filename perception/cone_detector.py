import carla
import numpy as np
from collections import deque
from sklearn.cluster import DBSCAN

from bilingual_console import configure_utf8_console


configure_utf8_console()


# 运行和跟踪参数 / Run and tracking settings
SIMULATION_FRAMES = 400
DETECT_EVERY_N_FRAMES = 5
POINT_HISTORY = deque(maxlen=2)
TRACKS = []
NEXT_TRACK_ID = 1

# LiDAR 局部坐标中的检测区。/ Detection area in LiDAR-local coordinates.
ROI_X_MIN = 3.0
ROI_X_MAX = 25.0
ROI_Y_ABS_MAX = 4.5
ROI_Z_MIN = -2.30
ROI_Z_MAX = -1.20

# 锥桶聚类的几何范围 / Geometric limits for a cone cluster
MIN_CONE_POINTS = 8
MAX_CONE_POINTS = 300
MIN_CONE_HORIZONTAL_SPAN = 0.30
MAX_CONE_HORIZONTAL_SPAN = 0.85
MIN_CONE_SECONDARY_SPAN = 0.08
MIN_CONE_HEIGHT = 0.20
MAX_CONE_HEIGHT = 1.10
MIN_CONE_TAPER_RATIO = 0.55
MAX_CONE_TAPER_RATIO = 3.00


def local_to_world(transform, xyz):
    """把 LiDAR 局部点转成世界点。/ Convert a LiDAR-local point to a world point."""
    location = carla.Location(
        x=float(xyz[0]),
        y=float(xyz[1]),
        z=float(xyz[2]),
    )
    transform.transform(location)
    return location


def measure_cone_cluster(cluster):
    """测量聚类的大小和锥形。/ Measure the cluster size and cone taper."""
    # 计算三轴范围。/ Measure the span on all three axes.
    size = cluster.max(axis=0) - cluster.min(axis=0)
    horizontal_max = float(max(size[0], size[1]))
    horizontal_min = float(min(size[0], size[1]))
    height = float(size[2])

    # 按高度分成上下两部分。/ Split the cluster into lower and upper parts.
    z_middle = float(cluster[:, 2].min() + height * 0.5)
    lower = cluster[cluster[:, 2] <= z_middle]
    upper = cluster[cluster[:, 2] > z_middle]

    # 真锥桶通常下宽上窄。/ A real cone is usually wider at the bottom.
    xy_center = cluster[:, :2].mean(axis=0)
    lower_radius = (
        float(
            np.mean(
                np.linalg.norm(lower[:, :2] - xy_center, axis=1)
            )
        )
        if len(lower) > 0
        else 0.0
    )
    upper_radius = (
        float(
            np.mean(
                np.linalg.norm(upper[:, :2] - xy_center, axis=1)
            )
        )
        if len(upper) > 0
        else 0.0
    )
    taper_ratio = lower_radius / max(upper_radius, 0.01)

    return {
        "point_count": len(cluster),
        "horizontal_max": horizontal_max,
        "horizontal_min": horizontal_min,
        "height": height,
        "taper_ratio": taper_ratio,
    }


def is_cone_cluster(cluster):
    """用点数、尺寸和锥形判断候选。/ Check a candidate by points, size, and taper."""
    features = measure_cone_cluster(cluster)
    is_cone = (
        MIN_CONE_POINTS
        <= features["point_count"]
        <= MAX_CONE_POINTS
        and MIN_CONE_HORIZONTAL_SPAN
        <= features["horizontal_max"]
        <= MAX_CONE_HORIZONTAL_SPAN
        and features["horizontal_min"]
        >= MIN_CONE_SECONDARY_SPAN
        and MIN_CONE_HEIGHT
        <= features["height"]
        <= MAX_CONE_HEIGHT
        and MIN_CONE_TAPER_RATIO
        <= features["taper_ratio"]
        <= MAX_CONE_TAPER_RATIO
    )
    return is_cone, features


def update_tracks(world, frame, candidate_locations):
    """合并连续帧候选，得到稳定轨迹。/ Merge frame candidates into stable tracks."""
    global TRACKS, NEXT_TRACK_ID

    # 记录本帧已经匹配的轨迹。/ Record tracks already matched in this frame.
    matched_track_indexes = set()

    for location in candidate_locations:
        candidate = np.array(
            [location.x, location.y, location.z],
            dtype=np.float32,
        )

        best_index = None
        best_distance = 0.75

        # 找到距离最近的已有轨迹。/ Find the nearest existing track.
        for index, track in enumerate(TRACKS):
            if index in matched_track_indexes:
                continue

            distance = float(
                np.linalg.norm(track["position"] - candidate)
            )
            if distance < best_distance:
                best_distance = distance
                best_index = index

        if best_index is None:
            # 没有匹配时创建新轨迹。/ Create a new track when there is no match.
            TRACKS.append(
                {
                    "id": NEXT_TRACK_ID,
                    "position": candidate,
                    "hits": 1,
                    "last_seen": frame,
                }
            )
            matched_track_indexes.add(len(TRACKS) - 1)
            NEXT_TRACK_ID += 1
        else:
            # 平滑更新已有轨迹。/ Smoothly update the existing track.
            track = TRACKS[best_index]
            track["position"] = (
                0.7 * track["position"] + 0.3 * candidate
            )
            track["hits"] += 1
            track["last_seen"] = frame
            matched_track_indexes.add(best_index)

    # 删除长时间未出现的轨迹。/ Remove tracks not seen for a long time.
    TRACKS = [
        track
        for track in TRACKS
        if frame - track["last_seen"] <= 30
    ]

    # 至少命中三次才算稳定。/ A track needs at least three hits to be stable.
    stable_tracks = [
        track
        for track in TRACKS
        if track["hits"] >= 3
        and frame - track["last_seen"] <= 10
    ]

    # 用红色点和编号显示稳定轨迹。/ Draw stable tracks with red points and IDs.
    for track in stable_tracks:
        position = track["position"]
        location = carla.Location(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
        )

        world.debug.draw_point(
            location,
            size=0.18,
            color=carla.Color(255, 0, 0),
            life_time=0.30,
            persistent_lines=False,
        )
        world.debug.draw_string(
            location + carla.Location(z=0.7),
            f"CONE T{track['id']}",
            draw_shadow=True,
            color=carla.Color(255, 0, 0),
            life_time=0.30,
            persistent_lines=False,
        )

    return len(stable_tracks)


def update_camera(world, vehicle):
    """让第三人称相机跟随车辆。/ Make the third-person camera follow the vehicle."""
    spectator = world.get_spectator()
    transform = vehicle.get_transform()
    location = transform.location
    forward = transform.get_forward_vector()

    camera_location = carla.Location(
        x=location.x - forward.x * 10.0,
        y=location.y - forward.y * 10.0,
        z=location.z + 6.0,
    )
    camera_rotation = carla.Rotation(
        pitch=-20.0,
        yaw=transform.rotation.yaw,
    )
    spectator.set_transform(
        carla.Transform(camera_location, camera_rotation)
    )


def detect_cones(world, lidar_data):
    """处理点云并检测锥桶。/ Process LiDAR points and detect cones."""
    # 读取当前帧并保存到短历史。/ Read this frame and save it in short history.
    raw_points = np.frombuffer(
        lidar_data.raw_data,
        dtype=np.float32,
    ).reshape(-1, 4)
    POINT_HISTORY.append(raw_points[:, :3].copy())

    if lidar_data.frame % DETECT_EVERY_N_FRAMES != 0:
        return

    if len(POINT_HISTORY) < 2:
        return

    # 合并两帧，近似得到完整一圈扫描。/ Merge two frames to approximate one full scan.
    xyz = np.concatenate(POINT_HISTORY, axis=0)

    # 1. 保留车辆前方检测区。/ 1. Keep the detection area in front of the car.
    roi_mask = (
        (xyz[:, 0] > ROI_X_MIN)
        & (xyz[:, 0] < ROI_X_MAX)
        & (np.abs(xyz[:, 1]) < ROI_Y_ABS_MAX)
        & (xyz[:, 2] > ROI_Z_MIN)
        & (xyz[:, 2] < ROI_Z_MAX)
    )
    roi_points = xyz[roi_mask]

    if len(roi_points) < 4:
        print(f"Frame / 帧 {lidar_data.frame}: insufficient ROI points / ROI点不足")
        return

    # 2. 用 DBSCAN 把邻近点分组。/ 2. Group nearby points with DBSCAN.
    labels = DBSCAN(
        eps=0.42,
        min_samples=2,
    ).fit_predict(roi_points)

    detected_locations = []
    cluster_count = 0

    for label in sorted(set(labels)):
        if label == -1:
            continue

        cluster = roi_points[labels == label]
        cluster_count += 1

        # 计算聚类的中心和大小。/ Measure the cluster center and size.
        center = cluster.mean(axis=0)

        # 3. 用点数、尺寸和锥形筛选。/ 3. Filter by points, size, and cone taper.
        is_cone, _ = is_cone_cluster(cluster)

        center_world = local_to_world(lidar_data.transform, center)

        # 候选用橙色，拒绝项用蓝色。/ Use orange for candidates and blue for rejected clusters.
        if is_cone:
            detected_locations.append(center_world)
            color = carla.Color(255, 120, 0)
            point_size = 0.10
        else:
            color = carla.Color(0, 180, 255)
            point_size = 0.07

        world.debug.draw_point(
            center_world,
            size=point_size,
            color=color,
            life_time=0.30,
            persistent_lines=False,
        )

    # 跨帧跟踪，减少单帧抖动。/ Track across frames to reduce frame-to-frame noise.
    stable_count = update_tracks(
        world,
        lidar_data.frame,
        detected_locations,
    )

    print(
        f"Frame / 帧 {lidar_data.frame}: raw / 原始 {len(xyz)}, "
        f"ROI {len(roi_points)}, clusters / 聚类 {cluster_count}, "
        f"frame candidates / 单帧候选 {len(detected_locations)}, "
        f"stable cones / 稳定锥桶 {stable_count}"
    )


def create_test_cones(world, blueprint_library, spawn_transform):
    """生成两排测试锥桶。/ Create two rows of test cones."""
    # 选择交通锥模型。/ Select the traffic-cone model.
    cone_blueprint = blueprint_library.find(
        "static.prop.trafficcone01"
    )
    cones = []

    # 设置相对车辆的位置。/ Set positions relative to the vehicle.
    offsets = []
    for distance in (6.0, 10.0, 14.0, 18.0, 22.0):
        offsets.append((distance, -2.2))
        offsets.append((distance, 2.2))

    # 转成世界坐标并生成锥桶。/ Convert to world coordinates and spawn the cones.
    for x, y in offsets:
        cone_location = carla.Location(x=x, y=y, z=0.05)
        spawn_transform.transform(cone_location)
        cone = world.try_spawn_actor(
            cone_blueprint,
            carla.Transform(
                cone_location,
                carla.Rotation(yaw=spawn_transform.rotation.yaw),
            ),
        )
        if cone is not None:
            cones.append(cone)

    return cones


def main():
    # 连接 CARLA 服务。/ Connect to the CARLA server.
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    # 读取世界，并保存原设置。/ Get the world and save its original settings.
    world = client.get_world()
    original_settings = world.get_settings()
    blueprint_library = world.get_blueprint_library()

    vehicle = None
    lidar = None
    cones = []

    try:
        # 使用固定 20 FPS 的同步模式。/ Use synchronous mode at a fixed 20 FPS.
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        # 在固定出生点生成并停住车辆。/ Spawn and hold the vehicle at a fixed point.
        spawn_transform = world.get_map().get_spawn_points()[0]
        vehicle_blueprint = blueprint_library.find(
            "vehicle.tesla.model3"
        )
        vehicle = world.try_spawn_actor(
            vehicle_blueprint,
            spawn_transform,
        )

        if vehicle is None:
            print("Vehicle spawn point is occupied; restart CARLA and retry. / 车辆生成位置被占用，请重启CARLA后再试。")
            return

        vehicle.apply_control(
            carla.VehicleControl(throttle=0.0, brake=1.0)
        )

        # 建立已知位置的测试场景。/ Build a test scene with known cone positions.
        cones = create_test_cones(
            world,
            blueprint_library,
            spawn_transform,
        )

        # 设置高密度 40 线 LiDAR。/ Configure a dense 40-channel LiDAR.
        lidar_blueprint = blueprint_library.find(
            "sensor.lidar.ray_cast"
        )
        lidar_blueprint.set_attribute("channels", "40")
        lidar_blueprint.set_attribute("range", "50")
        lidar_blueprint.set_attribute("points_per_second", "800000")
        lidar_blueprint.set_attribute("rotation_frequency", "10")
        lidar_blueprint.set_attribute("upper_fov", "15")
        lidar_blueprint.set_attribute("lower_fov", "-25")

        # 把 LiDAR 安装在车顶并开始检测。/ Mount the LiDAR on the roof and start detection.
        lidar = world.spawn_actor(
            lidar_blueprint,
            carla.Transform(carla.Location(z=2.4)),
            attach_to=vehicle,
        )
        lidar.listen(lambda data: detect_cones(world, data))

        print(f"Test cones created / 已生成测试锥桶：{len(cones)}")
        print("Orange points: current-frame cone candidates. / 橙色点：当前帧通过几何条件的锥桶候选。")
        print("Red CONE T labels: stable cross-frame cones. / 红色CONE T编号：跨帧确认的稳定锥桶。")
        print("Blue points: clusters rejected by geometry. / 蓝色点：被几何条件拒绝的聚类中心。")

        # 推进固定场景检测。/ Run the fixed-scene detection.
        for _ in range(SIMULATION_FRAMES):
            update_camera(world, vehicle)
            world.tick()

        print("Cone detection test completed. / 锥桶检测测试完成。")

    finally:
        # 无论是否报错，都删除对象并恢复设置。/ Always destroy actors and restore settings.
        if lidar is not None:
            lidar.stop()
            lidar.destroy()

        for cone in cones:
            cone.destroy()

        if vehicle is not None:
            vehicle.destroy()

        world.apply_settings(original_settings)
        print("Vehicle, LiDAR, and cones removed; CARLA settings restored. / 车辆、LiDAR和锥桶已清理，CARLA设置已恢复。")


if __name__ == "__main__":
    main()
