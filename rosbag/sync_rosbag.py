from iterate_rosbag import iterate_rosbag
from rclpy.serialization import serialize_message
import rosbag2_py

"""
TODO (later): think about the global misalignment since we don't know when the radar starts recording

this will attempt to minimize the local misalignment from frames
"""

start_time = # the time that the radar starts recording at
frame_period = 33.30  # ms

topic = "/imu"
topic_type = "sensor_msgs/msg/Imu"  # ros2 bag info <bag>
input_bag = "rosbag2_2024_05_01-15_19_52"  # TODO: EDIT
output_bag = "synced_" + input_bag
storage_id = "sqlite3"  # alternative is "mcap"


writer = rosbag2_py.SequentialWriter()
writer.open(
    rosbag2_py.StorageOptions(uri=output_bag, storage_id=storage_id),
    rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    ),
)

# create topic
writer.create_topic(
    rosbag2_py.TopicMetadata(
        name=topic, type=topic_type, serialization_format="cdr"
    )
)


# print out the average loss per second

start_time = 0
for i in range(10):
    msg = String()
    msg.data = f"Chatter #{i}"
    timestamp = start_time + (i * 100)
    writer.write("/chatter", serialize_message(msg), timestamp)


del writer
