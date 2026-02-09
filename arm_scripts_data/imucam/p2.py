import sys
import rosbag2_py
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

IMU_TOPIC = "/imu/imu_plugin/out"

def analyze_imu(bag_path):
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='sqlite3'
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}

    if IMU_TOPIC not in type_map:
        print(f"❌ Brak {IMU_TOPIC} w rosbagu")
        return

    imu_type = get_message(type_map[IMU_TOPIC])

    t, ax, ay, az, gx, gy, gz = [], [], [], [], [], [], []
    t0 = None

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic != IMU_TOPIC:
            continue

        msg = deserialize_message(data, imu_type)

        if t0 is None:
            t0 = ts
        time_s = (ts - t0) * 1e-9

        t.append(time_s)
        ax.append(msg.linear_acceleration.x)
        ay.append(msg.linear_acceleration.y)
        az.append(msg.linear_acceleration.z)
        gx.append(msg.angular_velocity.x)
        gy.append(msg.angular_velocity.y)
        gz.append(msg.angular_velocity.z)

    plt.figure(figsize=(12, 8))

    plt.subplot(2, 1, 1)
    plt.plot(t, ax, label="ax")
    plt.plot(t, ay, label="ay")
    plt.plot(t, az, label="az")
    plt.title("IMU – Linear Acceleration")
    plt.xlabel("Time [s]")
    plt.ylabel("m/s²")
    plt.legend()
    plt.grid()

    plt.subplot(2, 1, 2)
    plt.plot(t, gx, label="gx")
    plt.plot(t, gy, label="gy")
    plt.plot(t, gz, label="gz")
    plt.title("IMU – Angular Velocity")
    plt.xlabel("Time [s]")
    plt.ylabel("rad/s")
    plt.legend()
    plt.grid()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 imu_analysis.py <bag_path>")
        sys.exit(1)

    analyze_imu(sys.argv[1])
