#!/usr/bin/env python3

import rospy 
import rospkg

from hri_audition.srv import RecordCalData, RecordCalDataResponse

from audio_common_msgs.msg import AudioInfo, AudioDataStamped

class AudioCalNode:

    def __init__(self):
        rospy.init_node('cal_node')

        # Get parameters
        # audio_info = rospy.wait_for_message('audio_info', AudioInfo,timeout=10)
        # self.n_channels = audio_info.channels
        # self.sample_rate = audio_info.sample_rate
        # self.sample_fmt = audio_info.sample_format

        # audio topic
        self.rospack = rospkg.RosPack()
        self.pkg_path = self.rospack.get_path('hri_audition')
        self.cal_data_dir = self.pkg_path + '/data/calibration/'

        # Set up ROS subs, pubs, servers
        # self.audio_data_subscriber = rospy.Subscriber('audio_stamped', AudioDataStamped, self.audio_data_callback)
        # self.publisher = rospy.Publisher('audio_features', AudioFeatures, queue_size=10)
        self.record_data_srv = rospy.Service('record_cal_data', RecordCalData, self.handle_record_cal_data)

    def handle_record_cal_data(self, req):
        print("recording data")
        print("Got req: ")
        print(req)

        return RecordCalDataResponse()


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    audio_cal_node = AudioCalNode()
    audio_cal_node.run()