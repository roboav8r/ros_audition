import os
from joblib import load

import rospy
import rospkg
from hri_msgs.msg import AudioFeatures
from std_msgs.msg import String

class SceneClassifier:

    def __init__(self):
        # Set up ROS node, publishers, subscribers
        rospy.init_node('scene_classifier_node')
        self.feature_data_sub = rospy.Subscriber('/audio/audio_features', AudioFeatures, self.feature_data_callback)
        self.scene_class_pub = rospy.Publisher('scene_class', String, queue_size=10)

        # Initialize member variables
        self.rospack = rospkg.RosPack()
        self.pkg_path = self.rospack.get_path('hri_audition')
        self.model_path = self.pkg_path + '/data/models/'

        # Load classifier models
        # self.clfs = load_classifiers()

    # def load_classifiers(self):
    #     # Search through model folder
    #     for root, dirs, files in os.walk(self.feat_path,topdown=True):


    #     # Add each model to list of classifiers and return

    # def predict(self,feature_msg):
    #     # Convert feature message into vector
    #     # Predict class of feature vector
    #     # Return result

    def feature_data_callback(self, feature_msg):
        # Do something with the audio data message
        # Process the audio data and generate a string output
        output_data = "Feature data received"
        self.scene_class_pub.publish(output_data)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = SceneClassifier()
    node.run()