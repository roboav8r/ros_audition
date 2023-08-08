import os
from joblib import load
import numpy as np

import rospy
import rospkg
from hri_msgs.msg import AudioFeatures
from std_msgs.msg import String

class VoiceClassifier:

    def __init__(self):
        # Set up ROS node, publishers, subscribers
        rospy.init_node('voice_classifier_node')
        self.feature_data_sub = rospy.Subscriber('/audio/audio_features', AudioFeatures, self.feature_data_callback)
        self.voice_class_pub = rospy.Publisher('voice_id', String, queue_size=10)

        # Initialize member variables
        self.rospack = rospkg.RosPack()
        self.pkg_path = self.rospack.get_path('hri_audition')
        self.model_path = self.pkg_path + '/data/models/'

        # Initialize and load classifier models
        self.clfs = []
        self.clf_names = []
        self.load_models()

        # Initialize likelihoods
        self.mfcc_vec = np.zeros((1,12))
        self.likelihoods = np.ones(len(self.clf_names))/len(self.clf_names)

    def load_models(self):
        # Search through model folder
        for root, dirs, files in os.walk(self.model_path,topdown=True):
            for file in files:
                filepath = os.path.join(root,file)
                
                # Get type label of datafile
                type_dir = os.path.dirname(filepath)
                type_str = os.path.split(type_dir)[-1]

                # Get base filename and class label of datafile
                base = os.path.splitext(file)[0]
                ext = os.path.splitext(file)[-1]
                    
                if type_str=='voice' and ext=='.joblib':
                    # Load this model and add to classifiers list
                    self.clfs.append(load(filepath))
                    self.clf_names.append(base)

    def feature_data_callback(self, feature_msg):
        # Do something with the audio data message
        # Process the audio data and generate a string output
        self.mfcc_vec = np.array([feature_msg.MFCC])

        for ii in range(len(self.likelihoods)):
            # Compute weighted likelihood that this feature matches this model
            self.likelihoods[ii] = np.multiply(self.clfs[ii].predict_proba(self.mfcc_vec),self.clfs[ii].weights_).sum()

        self.likelihoods /= self.likelihoods.sum()
        self.max_ind = np.argmax(self.likelihoods)
        output_data = str(self.clf_names[self.max_ind]) + ": (" + str(self.likelihoods[self.max_ind]) + ")"

        self.voice_class_pub.publish(output_data)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = VoiceClassifier()
    node.run()