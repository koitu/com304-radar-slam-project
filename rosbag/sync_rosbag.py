from iterate_rosbag import iterate_rosbag
from rclpy.serialization import serialize_message
import rosbag2_py

"""
TODO (later): think about the global misalignment since we don't know when the radar starts recording

this will attempt to minimize the local misalignment from frames
"""

# we should be able to get the start time of recording via os.path.getctime(path)
# on linux only some filesystems will record the creation time (birth time)
# stat stream_thing_next_Raw_0.bin
# time.mktime(datetime(2024, 5, 1, 19, 42, 55, 926446).timetuple()) + 0.016071185
# time.mktime(datetime(2024, 5, 1, 19, 27, 42, 926446).timetuple()) + 0.521676231
start_time = 1714584462.521676231  # TODO: the time that the radar starts recording at
frame_period = 33.30  # ms
tot_frame_num = 800  # number of radar frames

topic = "/imu"
topic_type = "sensor_msgs/msg/Imu"  # ros2 bag info <bag>
input_bag = "rosbag2_2024_05_01-19_23_13"  # TODO: EDIT
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


if __name__ == "__main__":
    prev_timestamp = None
    prev_msg = None
    frame_num = 0
    avg_error = 0

    for timestamp, msg in iterate_rosbag(input_bag, topic, storage_id):
        cur_frame_time = start_time + frame_num * frame_period

        # once the timestamp of rosbag is past cur_frame_time compare previous and next timestamps
        # while we allow one imu frame to be attached to two radar frames that should not occur as long as imu frame
        #   has a higher refresh rate
        if timestamp/1e9 > cur_frame_time:
            if abs(timestamp/1e9 - cur_frame_time) < abs(prev_timestamp/1e9 - cur_frame_time):
                writer.write(topic, serialize_message(msg), timestamp)
                avg_error += (timestamp/1e9 - cur_frame_time)/tot_frame_num

            else:
                writer.write(topic, serialize_message(prev_msg), prev_timestamp)
                avg_error += (prev_timestamp/1e9 - cur_frame_time)/tot_frame_num

            frame_num += 1
            if frame_num >= tot_frame_num:  # done
                print(f"Average error: {avg_error}")
                del writer
                break

        prev_timestamp, prev_msg = (timestamp, msg)
