import sys
import math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt


def analyze_laserscan(bag_path, topic_name="/laser_scan"):
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

    if topic_name not in type_map:
        print(f"❌ Nie znaleziono topiku {topic_name} w bagu")
        print("Dostępne topiki:")
        for t in type_map:
            print(" ", t)
        return

    msg_type = get_message(type_map[topic_name])

    all_ranges = []
    nan_count = 0
    inf_count = 0
    total_count = 0

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != topic_name:
            continue

        msg = deserialize_message(data, msg_type)

        for r in msg.ranges:
            total_count += 1
            if math.isnan(r):
                nan_count += 1
            elif math.isinf(r):
                inf_count += 1
            else:
                all_ranges.append(r)

    all_ranges = np.array(all_ranges)

    print("\n=== ANALIZA LASER SCAN ===")
    print(f"Topic: {topic_name}")
    print(f"Łącznie próbek: {total_count}")
    print(f"Poprawne: {len(all_ranges)}")
    print(f"NaN: {nan_count}")
    print(f"Inf: {inf_count}")

    if len(all_ranges) == 0:
        print("❌ Brak poprawnych danych do analizy.")
        return

    print(f"Min:  {all_ranges.min():.3f} m")
    print(f"Max:  {all_ranges.max():.3f} m")
    print(f"Mean: {all_ranges.mean():.3f} m")
    print(f"Std:  {all_ranges.std():.3f} m")

    # Histogram
    plt.figure()
    plt.hist(all_ranges, bins=50)
    plt.xlabel("Odległość [m]")
    plt.ylabel("Liczba próbek")
    plt.title("Histogram pomiarów LaserScan")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Użycie: python3 analyze_laserscan.py <ścieżka_do_baga> [topic]")
        sys.exit(1)

    bag_path = sys.argv[1]
    topic = sys.argv[2] if len(sys.argv) > 2 else "/laser_scan"

    analyze_laserscan(bag_path, topic)

