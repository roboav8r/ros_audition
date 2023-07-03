#!/usr/bin/env python3

import torch
import torchaudio
import rospy
from std_msgs.msg import String
from hri_msgs.msg import AudioFeatures

class TorchAudioFeatExtract:
    def __init__(self, dev=0, fpc=512, fs=16000):
        self.device = dev  # Set the default value for the device
        self.fpc = fpc
        self.sample_rate = fs

        # Create FFMPEG streamer object
        self.streamer = torchaudio.io.StreamReader("hw:" + str(self.device), "alsa")

        # Create a source stream
        self.streamer.add_audio_stream(
            frames_per_chunk=self.fpc,
            filter_desc=f"aresample={self.sample_rate},anull,aformat=sample_fmts=fltp"
        )

        # Create a publisher
        self.publisher = rospy.Publisher('output_topic', String, queue_size=10)


    def publish_message(self, message):
        # Publish the message
        self.publisher.publish(message)

def main():
    # Initialize the ROS node
    rospy.init_node('audio_feat_ext_node')

    # Load parameters from the parameter server
    # TODO 

    # Create an instance of the RosAudioNode class
    audio_node = TorchAudioFeatExtract()

    rate = rospy.Rate(10)  # Publish at 10 Hz (every 0.1 seconds)

    while not rospy.is_shutdown():
        # Example usage: publish a message
        message = String()
        message.data = 'Hello, ROS!'
        audio_node.publish_message(message)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
