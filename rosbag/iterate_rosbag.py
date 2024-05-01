from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

# requires: ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap


def iterate_rosbag(input_bag, topic, storage_id):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map = {
            topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    # Set filter for topic of string type
    storage_filter = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(storage_filter)

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        yield timestamp, msg

    del reader


if __name__ == "__main__":
    from datetime import datetime

    topic = "/imu"
    input_bag = "rosbag2_2024_05_01-15_19_52"  # TODO: EDIT
    storage_id = "sqlite3"  # alternative is "mcap"

    msg_counter = 0

    for timestamp, msg in iterate_rosbag(input_bag, topic, storage_id):
        # dir(msg): ['angular_velocity', 'angular_velocity_covariance', 'get_fields_and_field_types', 'header', 'linear_acceleration', 'linear_acceleration_covariance', 'orientation', 'orientation_covariance']

        # print(timestamp, msg)
        # print(f"{timestamp}: {msg.orientation}")
        # print(f"{timestamp}: {msg.angular_velocity}")
        print(f"{timestamp}: {msg.linear_acceleration}")

        # print(f"{datetime.fromtimestamp(timestamp / 1e9)}")
        if msg_counter > 10:
              break

        msg_counter += 1

