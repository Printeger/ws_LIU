import rosbag
from uwb_array.msg import uwb_array
from sensor_msgs.msg import Range
import rospy
import numpy as np


def calculate_distance(pos1, pos2):
    return np.linalg.norm(np.array(pos1) - np.array(pos2))


def add_fake_uwb_measurements(input_bag_path, output_bag_path, ground_truth_file, anchor_positions, max_vis_dist):
    # Load ground truth positions from file
    ground_truth_positions = []
    with open(ground_truth_file, 'r') as f:
        for line in f:
            pos = list(map(float, line.strip().split()))
            ground_truth_positions.append(pos)

    # Open the input bag file
    with rosbag.Bag(input_bag_path, 'r') as input_bag:
        # Create the output bag file
        with rosbag.Bag(output_bag_path, 'w') as output_bag:
            for topic, msg, t in input_bag.read_messages():
                # Write the original message to the output bag
                output_bag.write(topic, msg, t)

                # Create a uwb_array message
                uwb_range_array_msg = uwb_array()
                uwb_range_array_msg.header.stamp = t

                # Add fake UWB measurements at each timestamp
                for i, anchor_pos in enumerate(anchor_positions):
                    # Calculate the range between ground truth position and fake anchor position
                    if len(ground_truth_positions) > 0:
                        # Assuming the first position is used for all timestamps
                        ground_truth_pos = ground_truth_positions[0]
                        distance = calculate_distance(
                            ground_truth_pos, anchor_pos)
                        if distance > max_vis_dist:
                            continue  # Skip this anchor if it exceeds the max distance
                    else:
                        distance = 5.0  # Default value if no ground truth position is available

                    uwb_range_msg = Range()
                    uwb_range_msg.header.stamp = t
                    # Anchor labels start from 1
                    uwb_range_msg.header.frame_id = f"uwb_anchor_{i + 1}"
                    uwb_range_msg.radiation_type = Range.ULTRASOUND
                    uwb_range_msg.field_of_view = 0.1
                    uwb_range_msg.min_range = 0.2
                    uwb_range_msg.max_range = 10.0
                    uwb_range_msg.range = distance

                    # Add the Range message to the array
                    uwb_range_array_msg.uwb_range.append(uwb_range_msg)

                # Write the uwb_array message to the output bag
                output_bag.write("/fake_uwb_ranges", uwb_range_array_msg, t)


if __name__ == "__main__":
    rospy.init_node('gen_fake_uwb')
    input_bag_path = '/path/to/input.bag'
    output_bag_path = '/path/to/output.bag'
    ground_truth_file = '/path/to/ground_truth.txt'
    max_vis_dist = 10.0  # Set the maximum distance

    # Manually set the positions of the fake UWB anchors
    anchor_positions = [
        [0.0, 0.0, 0.0],  # Position of anchor 1
        [1.0, 0.0, 0.0],  # Position of anchor 2
        [0.0, 1.0, 0.0],  # Position of anchor 3
        [1.0, 1.0, 0.0],  # Position of anchor 4
        [0.5, 0.5, 1.0]   # Position of anchor 5
    ]

    add_fake_uwb_measurements(input_bag_path, output_bag_path,
                              ground_truth_file, anchor_positions, max_vis_dist)
