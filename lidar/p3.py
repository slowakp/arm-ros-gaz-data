import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt


def plot_trajectory(bag_path, parent_frame="vehicle_blue/odom", child_frame="vehicle_blue/chassis"):
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

    if "/model/vehicle_blue/tf" not in type_map:
        print("❌ W bagu nie ma topiku /model/vehicle_blue/tf")
        return

    tf_type = get_message(type_map["/model/vehicle_blue/tf"])

    xs = []
    ys = []

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != "/model/vehicle_blue/tf":
            continue

        msg = deserialize_message(data, tf_type)

        for transform in msg.transforms:
            if (transform.header.frame_id == parent_frame and
                transform.child_frame_id == child_frame):

                x = transform.transform.translation.x
                y = transform.transform.translation.y
                xs.append(x)
                ys.append(y)

    if len(xs) == 0:
        print("❌ Nie znaleziono żadnych transformacji dla pary:")
        print(f"   {parent_frame} -> {child_frame}")
        print("Sprawdź nazwy ramek w: ros2 topic echo /model/vehicle_blue/tf")
        return

    print(f"✅ Wczytano {len(xs)} punktów trajektorii")

    plt.figure()
    plt.plot(xs, ys, marker='.')
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title(f"Trajektoria: {parent_frame} -> {child_frame}")
    plt.axis("equal")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Użycie: python3 plot_trajectory_tf.py <ścieżka_do_baga> [parent] [child]")
        sys.exit(1)

    bag_path = sys.argv[1]
    parent = sys.argv[2] if len(sys.argv) > 2 else "map"
    child = sys.argv[3] if len(sys.argv) > 3 else "base_link"

    plot_trajectory(bag_path, parent, child)

