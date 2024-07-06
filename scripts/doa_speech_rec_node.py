#!/usr/bin/env python3

import os
# import json
import yaml
import torch
import numpy as np
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile
from scipy.signal import fftconvolve
import IPython
import pyroomacoustics as pra

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from audio_common_msgs.msg import AudioDataStamped
from ros_audition.msg import AudioAzSources, SpeechAzSource, SpeechAzSources


class DirectionalSpeechRecNode(Node):

    def __init__(self):
        super().__init__('directional_speech_rec_node')
        self.subscription = self.create_subscription(
            AudioAzSources,
            'audio_az_sources',
            self.audio_data_callback,
            10)
        
        
        self.source_pub = self.create_publisher(SpeechAzSources, 'speech_az_sources', 10)

        self.source_msg = SpeechAzSource()
        self.sources_msg = SpeechAzSources()

        # Declare parameters with default values
        self.declare_parameter('n_channels', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('sample_rate', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('microphone_frame_id',rclpy.Parameter.Type.STRING)
        self.declare_parameter('frame_size', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('speed_sound',rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('n_fft', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('n_sources', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('f_min', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('f_max', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('array_x_pos',rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('array_y_pos',rclpy.Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('ssl_algo',rclpy.Parameter.Type.STRING)

        # Retrieve parameters
        self.n_channels = self.get_parameter('n_channels').get_parameter_value().integer_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.microphone_frame_id = self.get_parameter('microphone_frame_id').get_parameter_value().string_value
        self.frame_size = self.get_parameter('frame_size').get_parameter_value().integer_value
        self.speed_sound = self.get_parameter('speed_sound').get_parameter_value().double_value
        self.n_fft = self.get_parameter('n_fft').get_parameter_value().integer_value
        self.n_sources = self.get_parameter('n_sources').get_parameter_value().integer_value
        self.freq_range = [self.get_parameter('f_min').get_parameter_value().integer_value, self.get_parameter('f_max').get_parameter_value().integer_value]
        self.array_pos = np.array([self.get_parameter('array_x_pos').get_parameter_value().double_array_value,
                                   self.get_parameter('array_y_pos').get_parameter_value().double_array_value])
        self.ssl_algo = self.get_parameter('ssl_algo').get_parameter_value().string_value

        # Audio data storage
        self.time_domain_frames = []

    def audio_data_callback(self, msg):

        # for az in each source, match to existing audio stream
        self.get_logger().info("Got message")


def main(args=None):
    rclpy.init(args=args)
    speech_node = DirectionalSpeechRecNode()
    rclpy.spin(speech_node)
    
    speech_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
