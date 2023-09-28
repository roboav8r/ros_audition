import os

import rospy
import rospkg

from audio_common_msgs import AudioDataStamped
from std_msgs.msg import String

class SceneClassifier:

    def __init__(self):
        # Set up ROS node, publishers, subscribers
        self.feature_data_sub = rospy.Subscriber('/audio/audio_stamped', AudioDataStamped, self.audio_data_callback)
        self.scene_class_pub = rospy.Publisher('scene_class', String, queue_size=10)

        # Initialize member variables
        self.rospack = rospkg.RosPack()
        self.pkg_path = self.rospack.get_path('hri_audition')
        self.model_path = self.pkg_path + '/data/models/'

    def audio_data_callback(self, feature_msg):
        output_data = "Audio data received"
        self.scene_class_pub.publish(output_data)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('scene_classifier_node')
    node = SceneClassifier()
    node.run()