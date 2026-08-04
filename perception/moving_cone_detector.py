import argparse
import csv
import json
from collections import deque
from datetime import datetime
from pathlib import Path
from threading import Lock

import carla
import numpy as np
from sklearn.cluster import DBSCAN

import cone_detector as detector
from bilingual_console import configure_utf8_console


configure_utf8_console()


# 运行和匹配参数 / Run and matching settings
SIMULATION_FRAMES = 700
VEHICLE_SPEED_MPS = 2.0
DETECT_EVERY_N_FRAMES = 5
MATCH_DISTANCE_METERS = 0.80

# 共享数据与评分状态 / Shared data and evaluation state
CSV_LOCK = Lock()
WORLD_POINT_HISTORY = deque(maxlen=2)
EVALUATION = {
    "frames": 0,
    "true_positives": 0,
    "false_positives": 0,
    "false_negatives": 0,
    "position_errors": [],
}


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


def points_in_sensor_roi(world_points, sensor_transform):
    """保留 LiDAR 检测区内的世界点。/ Keep world points inside the LiDAR detection area."""
    if len(world_points) == 0:
        return np.empty((0, 3), dtype=np.float32)

    # 加上齐次坐标，便于矩阵变换。/ Add homogeneous coordinates for matrix transforms.
    homogeneous = np.column_stack(
        (world_points, np.ones(len(world_points), dtype=np.float32))
    )
    # 从世界坐标转回传感器局部坐标。/ Convert world points back to sensor-local points.
    inverse_matrix = np.asarray(
        sensor_transform.get_inverse_matrix(),
        dtype=np.float32,
    )
    local_points = (inverse_matrix @ homogeneous.T).T[:, :3]
    # 使用与检测器相同的 ROI。/ Use the same ROI as the detector.
    mask = (
        (local_points[:, 0] > detector.ROI_X_MIN)
        & (local_points[:, 0] < detector.ROI_X_MAX)
        & (np.abs(local_points[:, 1]) < detector.ROI_Y_ABS_MAX)
        & (local_points[:, 2] > detector.ROI_Z_MIN)
        & (local_points[:, 2] < detector.ROI_Z_MAX)
    )
    return world_points[mask]


def evaluate_tracks(frame, sensor_transform, stable_tracks, ground_truth):
    """把检测结果与 CARLA 真值匹配评分。/ Match detections to CARLA ground truth and score them."""
    # 只评估当前可见的真实锥桶。/ Evaluate only ground-truth cones visible now.
    visible_truth = points_in_sensor_roi(ground_truth, sensor_transform)

    # 只评估当前检测区内的稳定轨迹。/ Evaluate only stable tracks inside the current detection area.
    if stable_tracks:
        track_points = np.asarray(
            [track["position"] for track in stable_tracks],
            dtype=np.float32,
        )
        visible_tracks = points_in_sensor_roi(
            track_points,
            sensor_transform,
        )
    else:
        visible_tracks = np.empty((0, 3), dtype=np.float32)

    # 收集匹配距离以内的所有组合。/ Collect all pairs inside the matching distance.
    possible_matches = []
    for truth_index, truth in enumerate(visible_truth):
        for track_index, track in enumerate(visible_tracks):
            distance = float(np.linalg.norm(truth - track))
            if distance <= MATCH_DISTANCE_METERS:
                possible_matches.append(
                    (distance, truth_index, track_index)
                )

    # 从最近距离开始做一对一匹配。/ Make one-to-one matches from the shortest distance.
    matched_truth = set()
    matched_tracks = set()
    errors = []
    for distance, truth_index, track_index in sorted(possible_matches):
        if truth_index in matched_truth or track_index in matched_tracks:
            continue
        matched_truth.add(truth_index)
        matched_tracks.add(track_index)
        errors.append(distance)

    # 计算 TP、FP、FN 和位置误差。/ Calculate TP, FP, FN, and position error.
    true_positives = len(errors)
    false_positives = len(visible_tracks) - true_positives
    false_negatives = len(visible_truth) - true_positives
    precision = (
        true_positives / (true_positives + false_positives)
        if true_positives + false_positives > 0
        else 0.0
    )
    recall = (
        true_positives / (true_positives + false_negatives)
        if true_positives + false_negatives > 0
        else 0.0
    )
    mean_error = float(np.mean(errors)) if errors else 0.0

    # 累加整个运行过程的评分。/ Accumulate metrics for the full run.
    EVALUATION["frames"] += 1
    EVALUATION["true_positives"] += true_positives
    EVALUATION["false_positives"] += false_positives
    EVALUATION["false_negatives"] += false_negatives
    EVALUATION["position_errors"].extend(errors)

    return {
        "frame": frame,
        "visible_truth": len(visible_truth),
        "visible_tracks": len(visible_tracks),
        "true_positives": true_positives,
        "false_positives": false_positives,
        "false_negatives": false_negatives,
        "precision": precision,
        "recall": recall,
        "mean_error": mean_error,
    }


def build_evaluation_summary():
    """生成最终评分汇总。/ Build the final evaluation summary."""
    # 汇总所有已评分帧。/ Summarize all evaluated frames.
    tp = EVALUATION["true_positives"]
    fp = EVALUATION["false_positives"]
    fn = EVALUATION["false_negatives"]
    precision = tp / (tp + fp) if tp + fp > 0 else 0.0
    recall = tp / (tp + fn) if tp + fn > 0 else 0.0
    f1_score = (
        2.0 * precision * recall / (precision + recall)
        if precision + recall > 0
        else 0.0
    )
    errors = EVALUATION["position_errors"]
    return {
        "evaluated_frames": EVALUATION["frames"],
        "true_positives": tp,
        "false_positives": fp,
        "false_negatives": fn,
        "precision": round(precision, 4),
        "recall": round(recall, 4),
        "f1_score": round(f1_score, 4),
        "mean_position_error_m": round(float(np.mean(errors)), 4)
        if errors
        else None,
        "max_position_error_m": round(float(np.max(errors)), 4)
        if errors
        else None,
    }


def detect_moving_cones(
    world,
    lidar_data,
    detection_writer,
    detection_file,
    evaluation_writer,
    evaluation_file,
    ground_truth,
    show_frame_output,
):
    """在车辆移动时检测、跟踪并评分锥桶。/ Detect, track, and score cones while the car moves."""
    # 读取当前 LiDAR 点云。/ Read the current LiDAR point cloud.
    points = np.frombuffer(
        lidar_data.raw_data,
        dtype=np.float32,
    ).reshape(-1, 4)[:, :3]

    # 先在传感器局部坐标中裁剪 ROI。/ First crop the ROI in sensor-local coordinates.
    roi_mask = (
        (points[:, 0] > detector.ROI_X_MIN)
        & (points[:, 0] < detector.ROI_X_MAX)
        & (np.abs(points[:, 1]) < detector.ROI_Y_ABS_MAX)
        & (points[:, 2] > detector.ROI_Z_MIN)
        & (points[:, 2] < detector.ROI_Z_MAX)
    )
    local_roi_points = points[roi_mask]

    # 把 ROI 点转换到固定的世界坐标。/ Convert ROI points to fixed world coordinates.
    if len(local_roi_points) > 0:
        homogeneous = np.column_stack(
            (
                local_roi_points,
                np.ones(len(local_roi_points), dtype=np.float32),
            )
        )
        transform_matrix = np.asarray(
            lidar_data.transform.get_matrix(),
            dtype=np.float32,
        )
        world_roi_points = (
            transform_matrix @ homogeneous.T
        ).T[:, :3]
    else:
        world_roi_points = np.empty((0, 3), dtype=np.float32)

    # 保存两帧世界点，避免车辆移动造成重影。/ Keep two world-space frames to avoid motion ghosting.
    WORLD_POINT_HISTORY.append(world_roi_points)

    if lidar_data.frame % DETECT_EVERY_N_FRAMES != 0:
        return
    if len(WORLD_POINT_HISTORY) < 2:
        return

    roi_points = np.concatenate(WORLD_POINT_HISTORY, axis=0)
    if len(roi_points) < 4:
        return

    # 用 DBSCAN 把邻近点分组。/ Group nearby points with DBSCAN.
    labels = DBSCAN(eps=0.42, min_samples=2).fit_predict(roi_points)
    candidate_locations = []
    cluster_count = 0

    for label in sorted(set(labels)):
        if label == -1:
            continue

        # 用统一的锥形规则筛选候选。/ Filter candidates with the shared cone-shape rules.
        cluster = roi_points[labels == label]
        cluster_count += 1
        center = cluster.mean(axis=0)
        is_cone, _ = detector.is_cone_cluster(cluster)

        center_world = carla.Location(
            x=float(center[0]),
            y=float(center[1]),
            z=float(center[2]),
        )
        # 用橙色显示当前帧候选。/ Draw current-frame candidates in orange.
        if is_cone:
            candidate_locations.append(center_world)
            world.debug.draw_point(
                center_world,
                size=0.10,
                color=carla.Color(255, 120, 0),
                life_time=0.30,
                persistent_lines=False,
            )

    # 跨帧跟踪，得到稳定锥桶。/ Track across frames to get stable cones.
    stable_count = detector.update_tracks(
        world,
        lidar_data.frame,
        candidate_locations,
    )
    stable_tracks = [
        track
        for track in detector.TRACKS
        if track["hits"] >= 3
        and lidar_data.frame - track["last_seen"] <= 10
    ]

    # 与已知锥桶真值进行自动评分。/ Score against known cone ground truth.
    metrics = evaluate_tracks(
        lidar_data.frame,
        lidar_data.transform,
        stable_tracks,
        ground_truth,
    )

    # 保存检测坐标和逐帧评分。/ Save detection positions and frame metrics.
    with CSV_LOCK:
        for track in stable_tracks:
            position = track["position"]
            detection_writer.writerow(
                [
                    lidar_data.frame,
                    f"{lidar_data.timestamp:.3f}",
                    track["id"],
                    f"{position[0]:.3f}",
                    f"{position[1]:.3f}",
                    f"{position[2]:.3f}",
                ]
            )
        detection_file.flush()

        evaluation_writer.writerow(
            [
                metrics["frame"],
                metrics["visible_truth"],
                metrics["visible_tracks"],
                metrics["true_positives"],
                metrics["false_positives"],
                metrics["false_negatives"],
                f"{metrics['precision']:.4f}",
                f"{metrics['recall']:.4f}",
                f"{metrics['mean_error']:.4f}",
            ]
        )
        evaluation_file.flush()

    # 单次测试显示逐帧信息，批量测试可关闭。/ Show frame details for a single run; batch tests can hide them.
    if show_frame_output:
        print(
            f"Frame / 帧 {lidar_data.frame}: two-frame world ROI / 两帧世界ROI {len(roi_points)}, "
            f"clusters / 聚类 {cluster_count}, frame candidates / 单帧候选 {len(candidate_locations)}, "
            f"stable cones / 稳定锥桶 {stable_count} | "
            f"GT {metrics['visible_truth']} TP {metrics['true_positives']} "
            f"FP {metrics['false_positives']} FN {metrics['false_negatives']} | "
            f"P {metrics['precision']:.1%} R {metrics['recall']:.1%} "
            f"Err {metrics['mean_error']:.3f}m"
        )


def main(
    vehicle_speed_mps=VEHICLE_SPEED_MPS,
    simulation_frames=SIMULATION_FRAMES,
    show_frame_output=True,
):
    """按指定速度和帧数运行一次测试。/ Run one test at the requested speed and frame count."""
    # 连接 CARLA 服务。/ Connect to the CARLA server.
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    # 读取世界，并保存原设置。/ Get the world and save its original settings.
    world = client.get_world()
    original_settings = world.get_settings()
    blueprint_library = world.get_blueprint_library()

    # 为本次运行建立带时间戳的输出文件。/ Create timestamped output files for this run.
    output_directory = Path("output")
    output_directory.mkdir(exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    speed_tag = f"{vehicle_speed_mps:.1f}".replace(".", "p")
    run_tag = f"speed_{speed_tag}_{timestamp}"
    detection_path = output_directory / f"moving_cones_{run_tag}.csv"
    evaluation_path = output_directory / f"evaluation_{run_tag}.csv"
    summary_path = output_directory / f"evaluation_summary_{run_tag}.json"

    # 清空上一次运行留下的跟踪和评分状态。/ Clear tracking and evaluation state from the last run.
    detector.TRACKS = []
    detector.NEXT_TRACK_ID = 1
    WORLD_POINT_HISTORY.clear()
    EVALUATION.update(
        {
            "frames": 0,
            "true_positives": 0,
            "false_positives": 0,
            "false_negatives": 0,
            "position_errors": [],
        }
    )

    vehicle = None
    lidar = None
    cones = []

    # 打开检测和评分 CSV。/ Open the detection and evaluation CSV files.
    with detection_path.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as detection_file, evaluation_path.open(
        "w",
        newline="",
        encoding="utf-8-sig",
    ) as evaluation_file:
        # 写入 CSV 表头。/ Write the CSV headers.
        detection_writer = csv.writer(detection_file)
        detection_writer.writerow(
            ["frame", "timestamp", "track_id", "x", "y", "z"]
        )
        evaluation_writer = csv.writer(evaluation_file)
        evaluation_writer.writerow(
            [
                "frame",
                "visible_ground_truth",
                "visible_detections",
                "true_positives",
                "false_positives",
                "false_negatives",
                "precision",
                "recall",
                "mean_position_error_m",
            ]
        )

        try:
            # 使用固定 20 FPS 的同步模式。/ Use synchronous mode at a fixed 20 FPS.
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            world.apply_settings(settings)

            # 在固定出生点生成车辆。/ Spawn the vehicle at a fixed point.
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

            # 生成测试锥桶，并记录 CARLA 真值坐标。/ Create test cones and record CARLA ground-truth positions.
            cones = detector.create_test_cones(
                world,
                blueprint_library,
                spawn_transform,
            )

            # 推进一帧，让 CARLA 写入新锥桶的真实坐标。/ Tick once so CARLA updates the new cones' real transforms.
            world.tick()

            ground_truth = np.asarray(
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
            lidar.listen(
                lambda data: detect_moving_cones(
                    world,
                    data,
                    detection_writer,
                    detection_file,
                    evaluation_writer,
                    evaluation_file,
                    ground_truth,
                    show_frame_output,
                )
            )

            # 让车辆按本次测试速度匀速前进。/ Move at the speed selected for this test.
            vehicle.enable_constant_velocity(
                carla.Vector3D(
                    x=float(vehicle_speed_mps),
                    y=0.0,
                    z=0.0,
                )
            )

            print(f"Test cones created / 已生成测试锥桶：{len(cones)}")
            print(
                f"Vehicle speed / 车辆速度：{vehicle_speed_mps:.1f} m/s; "
                f"test length / 测试长度：{simulation_frames} frames / 帧"
            )
            print(f"Detection coordinates / 检测坐标：{detection_path}")
            print(f"Frame metrics / 逐帧评分：{evaluation_path}")

            # 推进移动检测与评分。/ Run moving detection and evaluation.
            for _ in range(simulation_frames):
                update_camera(world, vehicle)
                world.tick()

            print("Moving-cone detection and automatic scoring completed. / 移动锥桶检测与自动评分完成。")

        finally:
            # 无论是否报错，都删除对象并恢复设置。/ Always destroy actors and restore settings.
            if lidar is not None:
                lidar.stop()
                lidar.destroy()
            for cone in cones:
                cone.destroy()
            if vehicle is not None:
                vehicle.disable_constant_velocity()
                vehicle.destroy()
            world.apply_settings(original_settings)

            # 写出最终 JSON 评分汇总。/ Write the final JSON evaluation summary.
            summary = build_evaluation_summary()
            summary.update(
                {
                    "vehicle_speed_mps": vehicle_speed_mps,
                    "simulation_frames": simulation_frames,
                    "simulated_duration_s": round(
                        simulation_frames * 0.05,
                        3,
                    ),
                    "planned_distance_m": round(
                        simulation_frames * 0.05 * vehicle_speed_mps,
                        3,
                    ),
                }
            )
            summary_path.write_text(
                json.dumps(summary, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )

            print("Vehicle, LiDAR, and cones removed; CARLA settings restored. / 车辆、LiDAR和锥桶已清理，CARLA设置已恢复。")
            print(f"Detection CSV / 检测CSV：{detection_path}")
            print(f"Evaluation CSV / 评分CSV：{evaluation_path}")
            print(f"Evaluation summary / 评分汇总：{summary_path}")
            print(
                "Final metrics / 最终评分："
                f"Precision={summary['precision']:.1%}, "
                f"Recall={summary['recall']:.1%}, "
                f"F1={summary['f1_score']:.1%}, "
                f"mean position error / 平均位置误差={summary['mean_position_error_m']} m"
            )

    # Refresh display copies after an interactive run. / 交互式运行结束后刷新双语展示副本。
    if show_frame_output:
        try:
            from tools.export_bilingual_reports import export_all

            export_all(Path("output"), Path("output") / "bilingual")
            print("Bilingual report copies refreshed. / 双语报告副本已刷新。")
        except Exception as error:  # Reporting must not break detection. / 报告错误不能影响检测。
            print(f"Bilingual export warning / 双语导出警告：{error}")

    return {
        "summary": summary,
        "detection_path": str(detection_path),
        "evaluation_path": str(evaluation_path),
        "summary_path": str(summary_path),
    }


def parse_arguments():
    """读取命令行速度和帧数。/ Read speed and frame count from the command line."""
    parser = argparse.ArgumentParser(
        description="Run CARLA moving-cone detection and scoring. / 运行CARLA移动锥桶检测与自动评分。",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=VEHICLE_SPEED_MPS,
        help="Vehicle speed in m/s. / 车辆速度，单位m/s。",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=SIMULATION_FRAMES,
        help="Simulation frames at 20 FPS. / 仿真帧数，20 FPS。",
    )
    arguments = parser.parse_args()
    if arguments.speed <= 0:
        parser.error("--speed must be greater than 0. / --speed必须大于0。")
    if arguments.frames <= 0:
        parser.error("--frames must be greater than 0. / --frames必须大于0。")
    return arguments


if __name__ == "__main__":
    args = parse_arguments()
    main(
        vehicle_speed_mps=args.speed,
        simulation_frames=args.frames,
    )
