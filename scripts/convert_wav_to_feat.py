import rosbag

input_bag = rosbag.Bag('input.bag')
output_bag = rosbag.Bag('output.bag', 'w')

most_recent_imu = None
for topic, msg, t in input_bag.read_messages(topics=['rgb_image', 'depth', 'imu']):
    # Store the most recent IMU message
    if topic == 'imu':
        most_recent_imu = msg
    elif topic == 'rgb_image':
        output_bag.write('rgb_image', msg)
        output_bag.write('imu', most_recent_imu)

output_bag.close()
input_bag.close()