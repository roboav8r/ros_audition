#!/usr/bin/env python3

import os
# import json
import yaml
import torch
import torchaudio
import numpy as np
from pathlib import Path
import time

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


class AudioSource():
    
    def __init__(self,az,frame):
        self.azimuth = az
        self.n_silent = 0
        self.frame = frame

    def append_frame(self, new_frame):
        self.frame = torch.cat((self.frame,new_frame),0)

class DirectionalSpeechRecNode(Node):

    def __init__(self):
        super().__init__('directional_speech_rec_node')

        # ROS pubs and subs
        self.subscription = self.create_subscription(
            AudioAzSources,
            'audio_az_sources',
            self.audio_data_callback,
            10)
        
        self.source_pub = self.create_publisher(SpeechAzSources, 'speech_az_sources', 10)
        self.source_msg = SpeechAzSource()
        self.sources_msg = SpeechAzSources()

        # ROS parameters
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
        self.declare_parameter('n_silent_frames', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('trigger_level', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('trigger_time', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('search_time', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('allowed_gap', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('pre_trigger_time', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('min_voice_samples', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('src_match_thresh_rad', rclpy.Parameter.Type.DOUBLE)

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

        self.n_silent_frames = self.get_parameter('n_silent_frames').get_parameter_value().integer_value 
        self.trigger_level = self.get_parameter('trigger_level').get_parameter_value().double_value
        self.trigger_time = self.get_parameter('trigger_time').get_parameter_value().double_value
        self.search_time = self.get_parameter('search_time').get_parameter_value().double_value
        self.allowed_gap = self.get_parameter('allowed_gap').get_parameter_value().double_value
        self.pre_trigger_time = self.get_parameter('pre_trigger_time').get_parameter_value().double_value
        self.min_voice_samples = self.get_parameter('min_voice_samples').get_parameter_value().integer_value 
        self.src_match_thresh_rad = self.get_parameter('src_match_thresh_rad').get_parameter_value().double_value

        # Torch
        self.torch_device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.vad = torchaudio.transforms.Vad(self.sample_rate, 
                                             trigger_level=self.trigger_level, 
                                             trigger_time=self.trigger_time,
                                             search_time=self.search_time,
                                             allowed_gap=self.allowed_gap,
                                             pre_trigger_time=self.pre_trigger_time) 
        self.vad.to(self.torch_device)
        
        # Audio data storage
        self.voice_sources = []

    def audio_data_callback(self, msg):

        self.get_logger().info("Tracking %s sources" % len(self.voice_sources))

        new_voice_sources = []
        new_voice_azimuths = []
        new_voice_matches = []

        # For each incoming directional source
        for ii, az_source in enumerate(msg.sources):

            # self.get_logger().info('number of bytes in audio frame: %s' % len(az_source.audio.data))

            # Put data in a torch tensor
            self.frame = torch.frombuffer(az_source.audio.data,dtype=torch.float16)
            # torch.save(self.frame,'incoming_frame_%s.pt' % ii)

            # Check for voice activity 
            tic = time.time()
            self.voice_data = self.vad.forward(self.frame)
            toc = time.time()
            # self.get_logger().info('voice data length: %s\n' % (len(self.voice_data)))
            self.get_logger().info('voice data length: %s\n VAD took %s\n' % (len(self.voice_data), toc-tic))

            if len(self.voice_data) > self.min_voice_samples:
                new_voice_sources.append(AudioSource(az_source.azimuth,self.frame))
                new_voice_azimuths.append(az_source.azimuth)

        self.get_logger().info("Got %s new sources" % len(new_voice_sources))

        # Match existing voice_sources with new voice sources
        for jj, voice in enumerate(self.voice_sources):

            # Compute distance between existing voice source and incoming voice sources
            az_dist = np.array(new_voice_azimuths) - voice.azimuth

            self.get_logger().info("Source at %s has original az_dist: %s" % (voice.azimuth, az_dist))

            for ii, az in enumerate(az_dist):
                if az > np.pi:
                    az_dist[ii] = az - 2*np.pi
                if az < -np.pi:
                    az_dist[ii] = az + 2*np.pi
            self.get_logger().info("Corrected az_dist: %s" % (az_dist))

            if len(az_dist) > 0: # If there are new sources
                min_idx = np.argmin(az_dist)
                self.get_logger().info("Closest source is %s with delta_angle %s" % (min_idx, az_dist[min_idx]))

                if az_dist[min_idx] < self.src_match_thresh_rad:
                    voice.append_frame(new_voice_sources[min_idx].frame)
                    new_voice_matches.append(min_idx)
                    # TODO - update azimuth of source

                else: # If unmatched old source, process, delete, and send message
                    # TODO - voice recognition
                    torch.save(voice.frame,'voice_frame_%s.pt' % ii)
                    del self.voice_sources[jj]

            else: # If unmatched old source, process, delete, and send message
                # TODO - voice recognition
                torch.save(voice.frame,'voice_frame_%s.pt' % ii)
                del self.voice_sources[jj]


        # Create voice sources from any new, unmatched voice sources 
        for ii, source in enumerate(new_voice_sources):
            if ii not in new_voice_matches:
                self.voice_sources.append(source)

def main(args=None):
    rclpy.init(args=args)
    speech_node = DirectionalSpeechRecNode()
    rclpy.spin(speech_node)
    
    speech_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
