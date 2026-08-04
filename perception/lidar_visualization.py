import carla
import numpy as np

from bilingual_console import configure_utf8_console


configure_utf8_console()


# 运行参数 / Run settings
CAMERA_MODE = "third_person"
SIMULATION_FRAMES = 600
DRAW_EVERY_NTH_POINT = 20


def update_spectator_camera(world, vehicle):
    """让观察相机跟随车辆。/ Make the spectator camera follow the vehicle."""
    spectator = world.get_spectator()
    transform = vehicle.get_transform()
    location = transform.location
    rotation = transform.rotation
    forward = transform.get_forward_vector()

    if CAMERA_MODE == "first_person":
        # 第一人称视角 / First-person view
        camera_location = carla.Location(
            x=location.x + forward.x * 1.2,
            y=location.y + forward.y * 1.2,
            z=location.z + 1.5,
        )
        camera_rotation = carla.Rotation(
            pitch=rotation.pitch,
            yaw=rotation.yaw,
            roll=rotation.roll,
        )
    elif CAMERA_MODE == "top_down":
        # 俯视视角 / Top-down view
        camera_location = carla.Location(
            x=location.x,
            y=location.y,
            z=location.z + 20.0,
        )
        camera_rotation = carla.Rotation(
            pitch=-90.0,
            yaw=rotation.yaw,
        )
    else:
        # 第三人称视角 / Third-person view
        camera_location = carla.Location(
            x=location.x - forward.x * 8.0,
            y=location.y - forward.y * 8.0,
            z=location.z + 4.0,
        )
        camera_rotation = carla.Rotation(
            pitch=-15.0,
            yaw=rotation.yaw,
        )

    spectator.set_transform(
        carla.Transform(camera_location, camera_rotation)
    )


def draw_lidar_points(world, lidar_data):
    """读取并绘制一帧点云。/ Read and draw one LiDAR frame."""
    # 把传感器字节转换为点数组。/ Convert sensor bytes into a point array.
    points = np.frombuffer(
        lidar_data.raw_data,
        dtype=np.float32,
    ).reshape(-1, 4)

    # 抽样以减少绘图量。/ Sample points to reduce drawing work.
    sampled_points = points[::DRAW_EVERY_NTH_POINT, :3]
    sensor_transform = lidar_data.transform

    # 把局部坐标转换到世界坐标并画成绿点。/ Convert local points to world points and draw them green.
    for x, y, z in sampled_points:
        point_location = carla.Location(
            x=float(x),
            y=float(y),
            z=float(z),
        )
        sensor_transform.transform(point_location)

        world.debug.draw_point(
            point_location,
            size=0.06,
            color=carla.Color(0, 255, 0),
            life_time=0.12,
            persistent_lines=False,
        )

    if lidar_data.frame % 20 == 0:
        print(
            f"Frame / 帧 {lidar_data.frame}: raw points / 原始点 {len(points)}, "
            f"displayed points / 显示点 {len(sampled_points)}"
        )


def main():
    # 连接 CARLA 服务。/ Connect to the CARLA server.
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    # 读取世界，并保存原设置。/ Get the world and save its original settings.
    world = client.get_world()
    original_settings = world.get_settings()
    blueprint_library = world.get_blueprint_library()
    traffic_manager = client.get_trafficmanager()

    vehicle = None
    lidar = None

    try:
        # 使用固定 20 FPS 的同步模式。/ Use synchronous mode at a fixed 20 FPS.
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        traffic_manager.set_synchronous_mode(True)

        # 选择 Tesla Model 3。/ Select a Tesla Model 3.
        vehicle_blueprint = blueprint_library.find(
            "vehicle.tesla.model3"
        )

        # 尝试每个出生点，直到车辆生成成功。/ Try spawn points until the vehicle is created.
        for spawn_point in world.get_map().get_spawn_points():
            vehicle = world.try_spawn_actor(
                vehicle_blueprint,
                spawn_point,
            )
            if vehicle is not None:
                break

        if vehicle is None:
            print("No free vehicle spawn point found. / 找不到空闲的车辆生成位置。")
            return

        # 设置 40 线 LiDAR 参数。/ Configure the 40-channel LiDAR.
        lidar_blueprint = blueprint_library.find(
            "sensor.lidar.ray_cast"
        )
        lidar_blueprint.set_attribute("channels", "40")
        lidar_blueprint.set_attribute("range", "50")
        lidar_blueprint.set_attribute("points_per_second", "200000")
        lidar_blueprint.set_attribute("rotation_frequency", "10")
        lidar_blueprint.set_attribute("upper_fov", "15")
        lidar_blueprint.set_attribute("lower_fov", "-25")

        # 把 LiDAR 安装在车顶。/ Mount the LiDAR on the roof.
        lidar = world.spawn_actor(
            lidar_blueprint,
            carla.Transform(carla.Location(z=2.4)),
            attach_to=vehicle,
        )

        # 开始接收点云，并开启自动驾驶。/ Start receiving points and enable autopilot.
        lidar.listen(lambda data: draw_lidar_points(world, data))
        vehicle.set_autopilot(True)

        print("Vehicle and 40-channel LiDAR created. / 车辆和40线LiDAR已创建。")
        print("Green points: LiDAR returns from surfaces. / 绿色点：LiDAR命中的物体表面。")
        print("Red point: LiDAR roof mounting position. / 红色点：LiDAR在车顶的安装位置。")
        print("Running for about 30 seconds. / 开始运行约30秒。")

        # 推进仿真，同时更新相机和传感器标记。/ Run the simulation while updating the camera and sensor marker.
        for _ in range(SIMULATION_FRAMES):
            update_spectator_camera(world, vehicle)

            lidar_location = lidar.get_transform().location
            world.debug.draw_point(
                lidar_location,
                size=0.15,
                color=carla.Color(255, 0, 0),
                life_time=0.12,
                persistent_lines=False,
            )

            world.tick()

        print("LiDAR visualization test completed. / LiDAR可视化测试完成。")

    finally:
        # 无论是否报错，都删除对象并恢复设置。/ Always destroy actors and restore settings.
        if lidar is not None:
            lidar.stop()
            lidar.destroy()
            print("LiDAR removed. / LiDAR已删除。")

        if vehicle is not None:
            vehicle.destroy()
            print("Vehicle removed. / 车辆已删除。")

        traffic_manager.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        print("CARLA settings restored. / CARLA设置已恢复。")


if __name__ == "__main__":
    main()
