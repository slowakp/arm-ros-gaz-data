import sys
import rosbag2_py
import numpy as np
import cv2
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

IMAGE_TOPIC = "/camera/camera/image_raw"

def show_frames(bag_path, t1, t2):
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

    if IMAGE_TOPIC not in type_map:
        print(f"❌ Brak {IMAGE_TOPIC} w rosbagu")
        return

    img_type = get_message(type_map[IMAGE_TOPIC])

    frames = []
    targets = [t1, t2]
    t0 = None

    while reader.has_next() and targets:
        topic, data, ts = reader.read_next()
        if topic != IMAGE_TOPIC:
            continue

        if t0 is None:
            t0 = ts
        time_s = (ts - t0) * 1e-9

        for target in targets:
            if abs(time_s - target) < 0.03:
                msg = deserialize_message(data, img_type)
                img = np.frombuffer(msg.data, dtype=np.uint8)
                img = img.reshape(msg.height, msg.width, -1)
                frames.append(img)
                targets.remove(target)
                break

    if len(frames) < 2:
        print("❌ Nie znaleziono obu ramek dla zadanych czasów")
        return

    combined = np.hstack(frames)
    cv2.imshow("Camera frames comparison", combined)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    if len(sys.argv) < 4:
        print("Usage: python3 camera_frame_compare.py <bag_path> <t1> <t2>")
        sys.exit(1)

    show_frames(sys.argv[1], float(sys.argv[2]), float(sys.argv[3]))
