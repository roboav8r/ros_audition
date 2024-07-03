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
from ros_audition.msg import AudioAzSource, AudioAzSources


class PyRoomAcousticsNode(Node):

    def __init__(self):
        super().__init__('pyroomaudio_node')
        self.subscription = self.create_subscription(
            AudioDataStamped,
            'audio_data',
            self.audio_data_callback,
            10)
        
        self.source_pub = self.create_publisher(AudioAzSources, 'audio_az_sources', 10)
        self.source_msg = AudioAzSource()
        self.sources_msg = AudioAzSources()

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
        self.time_domain_frame = torch.zeros([self.frame_size, self.n_channels],dtype=torch.float16)
        self.freq_domain_frame = torch.zeros([self.n_channels, self.n_fft, self.frame_size],dtype=torch.float16)

        # Audio processing
        self.doa = pra.doa.algorithms[self.ssl_algo](self.array_pos, self.sample_rate, self.n_fft, c=self.speed_sound, num_src=self.n_sources, max_four=4)

    def audio_data_callback(self, msg):

        chunk = torch.from_numpy(np.frombuffer(msg.audio.data,dtype=np.float16)).view(-1,self.n_channels)

        # Roll the frame, and replace oldest contents with new chunk
        self.time_domain_frame = torch.roll(self.time_domain_frame, -chunk.size(0), 0)
        self.time_domain_frame[-chunk.size(0):,:] = -chunk

        torch.save(self.time_domain_frame,'pra_node_data_recovered.pt')

        self.freq_domain_frame = pra.transform.stft.analysis(self.time_domain_frame, self.n_fft, self.n_fft//2).transpose([2, 1, 0])
        
        self.doa.locate_sources(self.freq_domain_frame, freq_range=self.freq_range)

        peaks = self.doa.grid.find_peaks(self.n_sources)


        # Format output
        self.sources_msg = AudioAzSources()
        self.sources_msg.header.stamp = msg.header.stamp
        self.sources_msg.header.frame_id = self.microphone_frame_id
        
        for peak in peaks:
            self.source_msg = AudioAzSource()
            self.source_msg.azimuth = self.doa.grid.azimuth[peak]
            self.source_msg.magnitude = self.doa.grid.values[peak]
            self.sources_msg.sources.append(self.source_msg)
            self.source_pub.publish(self.sources_msg)

def main(args=None):
    rclpy.init(args=args)
    pra_node = PyRoomAcousticsNode()
    rclpy.spin(pra_node)
    
    pra_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
