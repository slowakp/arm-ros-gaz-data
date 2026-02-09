import sys
import rosbag2_py
from collections import defaultdict

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

    topics_info = reader.get_all_topics_and_types()
    topic_types = {t.name: t.type for t in topics_info}

    counts = defaultdict(int)
    t_start = {}
    t_end = {}

    while reader.has_next():
        topic, data, t = reader.read_next()
        counts[topic] += 1
        if topic not in t_start:
            t_start[topic] = t
        t_end[topic] = t

    print("\n=== ROSBAG METADATA ANALYSIS ===\n")
    for topic in counts:
        duration = (t_end[topic] - t_start[topic]) * 1e-9
        freq = counts[topic] / duration if duration > 0 else 0.0

        print(f"Topic: {topic}")
        print(f"  Type: {topic_types.get(topic, 'unknown')}")
        print(f"  Messages: {counts[topic]}")
        print(f"  Duration: {duration:.2f} s")
        print(f"  Approx. frequency: {freq:.2f} Hz\n")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 bag_metadata_analysis.py <bag_path>")
        sys.exit(1)

    analyze_bag(sys.argv[1])
