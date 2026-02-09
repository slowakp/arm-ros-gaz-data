import sys
from collections import defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def analyze_bag(bag_path):
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

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    msg_count = defaultdict(int)
    first_time = {}
    last_time = {}

    while reader.has_next():
        topic, data, t = reader.read_next()
        msg_count[topic] += 1

        if topic not in first_time:
            first_time[topic] = t
        last_time[topic] = t

    print("\n=== PODSUMOWANIE ROSBAG ===\n")
    for topic in sorted(msg_count.keys()):
        n = msg_count[topic]
        t0 = first_time[topic] * 1e-9
        t1 = last_time[topic] * 1e-9
        duration = t1 - t0 if n > 1 else 0.0

        print(f"Topic: {topic}")
        print(f"  Typ: {type_map.get(topic, 'unknown')}")
        print(f"  Liczba wiadomości: {n}")
        print(f"  Czas: {t0:.3f} s -> {t1:.3f} s  (Δ = {duration:.3f} s)")
        if duration > 0:
            print(f"  Średnia częstotliwość: {n/duration:.2f} Hz")
        print()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Użycie: python3 analyze_bag.py <ścieżka_do_baga>")
        sys.exit(1)

    analyze_bag(sys.argv[1])

