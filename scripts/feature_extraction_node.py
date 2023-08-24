#!/usr/bin/env python3

import torch
import torchaudio
import librosa
import rospy
import numpy as np

from audio_common_msgs.msg import AudioInfo, AudioDataStamped
from hri_msgs.msg import AudioFeatures

class AudioFeatExtNode:

    def __init__(self):

        # Get parameters
        # audio_info = rospy.wait_for_message('audio_info', AudioInfo,timeout=10)
        self.n_channels = rospy.get_param("n_channels")
        self.sample_rate = rospy.get_param("sample_rate")
        self.sample_fmt = rospy.get_param("sample_fmt")
        self.channel_idx = rospy.get_param("channel_idx")
        self.frame_length = rospy.get_param("frame_length")
        self.hop_length = rospy.get_param("hop_length")

        # Torch/CUDA params
        self.torch_device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # MFCC transform params
        self.n_mfcc = 12 # Only want MFCCs 1-13, but we will compute 40
        self.mfcc_args = {"n_fft": self.frame_length, "hop_length": 2*self.frame_length, "n_mels": 40}
        self.mfcc_transform = torchaudio.transforms.MFCC(self.sample_rate, n_mfcc=40, melkwargs=self.mfcc_args)
        self.mfcc_data = torch.zeros([1,self.n_mfcc])
        self.mfcc_data.to(self.torch_device)

        # Initialize member variables/methods
        self.frame = torch.zeros([self.n_channels,self.frame_length])
        self.hop = torch.zeros([self.n_channels,self.hop_length])
        self.pitchdata = torch.zeros([1])
        self.frame.to(self.torch_device)
        self.hop.to(self.torch_device)
        self.pitchdata.to(self.torch_device)
        self.feature_msg = AudioFeatures()

        # Set up ROS subs and pubs
        self.audio_data_subscriber = rospy.Subscriber('audio_stamped', AudioDataStamped, self.audio_data_callback)
        self.publisher = rospy.Publisher('audio_features', AudioFeatures, queue_size=10)
        

    def extract_features(self):
        self.feature_msg.ZCR = (torch.sum(torch.diff(self.frame > 0),1)/self.frame_length)[0].item()
        self.feature_msg.RMS = torch.sqrt(torch.sum(torch.square(self.frame))/self.frame_length).item()
        # self.pitchdata = torchaudio.functional.detect_pitch_frequency(waveform=self.frame, sample_rate=self.sample_rate, frame_time=self.frame_length/self.sample_rate, win_length=1)
        self.pitchdata = torchaudio.functional.detect_pitch_frequency(self.frame, self.sample_rate)
        print(self.pitchdata)
        # self.feature_msg.pitch = torch.sum(self.pitchdata)/self.pitchdata.size()[1]
        self.feature_msg.pitch =3.0
        self.feature_msg.HNR = 4.0
        self.mfcc_data = self.mfcc_transform(self.frame)[0][1:self.n_mfcc+1,:]
        self.feature_msg.MFCC = self.mfcc_data

    def audio_data_callback(self, audio_data):
        # Save incoming audio data to hop tensor, casting from bytes type as needed
        self.hop = torch.from_numpy(librosa.util.buf_to_float(np.frombuffer(audio_data.audio.data, dtype=np.uint8), n_bytes=2)).view(self.n_channels,-1)

        # Roll the frame and replace old hop data with new hop data
        self.frame = torch.roll(self.frame, -self.hop_length, 1)
        self.frame[:,-self.hop_length:] = self.hop

        # Compute features of the latest frame
        self.extract_features()

        # Process the audio data and generate a string output
        self.publisher.publish(self.feature_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('feat_ext_node')
    node = AudioFeatExtNode()
    node.run()
