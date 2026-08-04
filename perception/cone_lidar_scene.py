import carla
import numpy as np

from bilingual_console import configure_utf8_console


configure_utf8_console()


# 运行参数 / Run settings
SIMULATION_FRAMES = 400
DRAW_EVERY_NTH_POINT = 10


def local_to_world(base_transform, x, y, z=0.05):
    """把车辆局部坐标转成世界坐标。/ Convert vehicle-local coordinates to world coordinates."""
    location = carla.Location(x=x, y=y, z=z)
    base_transform.transform(location)
    return location


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


def draw_lidar_points(world, lidar_data):
    """读取并按区域着色点云。/ Read and color LiDAR points by region."""
    # 把传感器字节转换为点数组。/ Convert sensor bytes into a point array.
    points = np.frombuffer(
        lidar_data.raw_data,
        dtype=np.float32,
    ).reshape(-1, 4)

    # 抽样以减少绘图量。/ Sample points to reduce drawing work.
    sampled = points[::DRAW_EVERY_NTH_POINT, :3]
    sensor_transform = lidar_data.transform

    for x, y, z in sampled:
        point_location = carla.Location(
            x=float(x),
            y=float(y),
            z=float(z),
        )
        sensor_transform.transform(point_location)

        # 测试区用黄色，其他区域用绿色。/ Use yellow in the test area and green elsewhere.
        in_test_roi = 2.0 < x < 25.0 and abs(y) < 5.0 and -3.0 < z < 1.0
        color = (
            carla.Color(255, 255, 0)
            if in_test_roi
            else carla.Color(0, 255, 0)
        )

        world.debug.draw_point(
            point_location,
            size=0.07,
            color=color,
            life_time=0.12,
            persistent_lines=False,
        )

    if lidar_data.frame % 20 == 0:
        print(f"Frame / 帧 {lidar_data.frame}: received / 接收到 {len(points)} points / 个点")


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

        # 选择交通锥模型。/ Select the traffic-cone model.
        cone_blueprint = blueprint_library.find(
            "static.prop.trafficcone01"
        )

        # 在车前生成两排锥桶。/ Create two rows of cones in front of the car.
        cone_offsets = []
        for distance in (6.0, 10.0, 14.0, 18.0, 22.0):
            cone_offsets.append((distance, -2.2))
            cone_offsets.append((distance, 2.2))

        # 生成锥桶并显示编号。/ Spawn the cones and show their labels.
        for index, (x, y) in enumerate(cone_offsets, start=1):
            cone_location = local_to_world(spawn_transform, x, y)
            cone_transform = carla.Transform(
                cone_location,
                carla.Rotation(yaw=spawn_transform.rotation.yaw),
            )
            cone = world.try_spawn_actor(cone_blueprint, cone_transform)
            if cone is not None:
                cones.append(cone)
                world.debug.draw_string(
                    cone_location + carla.Location(z=1.0),
                    f"C{index}",
                    draw_shadow=True,
                    color=carla.Color(255, 80, 0),
                    life_time=25.0,
                    persistent_lines=False,
                )

        # 设置 40 线 LiDAR 参数。/ Configure the 40-channel LiDAR.
        lidar_blueprint = blueprint_library.find(
            "sensor.lidar.ray_cast"
        )
        lidar_blueprint.set_attribute("channels", "40")
        lidar_blueprint.set_attribute("range", "50")
        lidar_blueprint.set_attribute("points_per_second", "400000")
        lidar_blueprint.set_attribute("rotation_frequency", "10")
        lidar_blueprint.set_attribute("upper_fov", "15")
        lidar_blueprint.set_attribute("lower_fov", "-25")

        # 把 LiDAR 安装在车顶并开始监听。/ Mount the LiDAR on the roof and start listening.
        lidar = world.spawn_actor(
            lidar_blueprint,
            carla.Transform(carla.Location(z=2.4)),
            attach_to=vehicle,
        )
        lidar.listen(lambda data: draw_lidar_points(world, data))

        print(f"Traffic cones created / 已生成交通锥：{len(cones)}")
        print("Yellow points: LiDAR points inside the front test area. / 黄色点：车辆前方测试区域内的LiDAR点。")
        print("Green points: LiDAR points outside the test area. / 绿色点：测试区域外的LiDAR点。")
        print("Starting a fixed-scene scan (about 20 s). / 开始固定场景扫描（约20秒）。")

        # 推进固定场景扫描。/ Run the fixed-scene scan.
        for _ in range(SIMULATION_FRAMES):
            update_camera(world, vehicle)
            world.tick()

        print("Cone LiDAR scene test completed. / 锥桶LiDAR场景测试完成。")

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
