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
from torchaudio.models.decoder import download_pretrained_files, ctc_decoder
import torchaudio.transforms as T
import sentencepiece as spm

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
        self.declare_parameter('sample_rate', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('microphone_frame_id',rclpy.Parameter.Type.STRING)
        self.declare_parameter('trigger_level', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('trigger_time', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('search_time', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('allowed_gap', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('pre_trigger_time', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('min_voice_samples', rclpy.Parameter.Type.INTEGER)
        self.declare_parameter('src_match_thresh_rad', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('lexicon_package', rclpy.Parameter.Type.STRING)
        self.declare_parameter('lexicon_file', rclpy.Parameter.Type.STRING)
        self.declare_parameter('am_bundle', rclpy.Parameter.Type.STRING)
        self.declare_parameter('lm_weight', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('word_score', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('sil_score', rclpy.Parameter.Type.DOUBLE)

        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.microphone_frame_id = self.get_parameter('microphone_frame_id').get_parameter_value().string_value

        self.trigger_level = self.get_parameter('trigger_level').get_parameter_value().double_value
        self.trigger_time = self.get_parameter('trigger_time').get_parameter_value().double_value
        self.search_time = self.get_parameter('search_time').get_parameter_value().double_value
        self.allowed_gap = self.get_parameter('allowed_gap').get_parameter_value().double_value
        self.pre_trigger_time = self.get_parameter('pre_trigger_time').get_parameter_value().double_value
        self.min_voice_samples = self.get_parameter('min_voice_samples').get_parameter_value().integer_value 
        self.src_match_thresh_rad = self.get_parameter('src_match_thresh_rad').get_parameter_value().double_value
        self.lexicon_package = self.get_parameter('lexicon_package').get_parameter_value().string_value
        self.lexicon_file = self.get_parameter('lexicon_file').get_parameter_value().string_value
        self.am_bundle = self.get_parameter('am_bundle').get_parameter_value().string_value
        self.lm_weight = self.get_parameter('lm_weight').get_parameter_value().double_value
        self.word_score = self.get_parameter('word_score').get_parameter_value().double_value
        self.sil_score = self.get_parameter('sil_score').get_parameter_value().double_value

        # Torch, voice activity and speech detection
        self.torch_device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        self.vad = torchaudio.transforms.Vad(self.sample_rate, 
                                             trigger_level=self.trigger_level, 
                                             trigger_time=self.trigger_time,
                                             search_time=self.search_time,
                                             allowed_gap=self.allowed_gap,
                                             pre_trigger_time=self.pre_trigger_time) 
        self.vad.to(self.torch_device)

        self.url_prefix = "https://huggingface.co/Zengwei/icefall-asr-librispeech-pruned-transducer-stateless7-ctc-2022-12-01"
        self.model_link = f"{self.url_prefix}/resolve/main/exp/cpu_jit.pt"
        self.model_path = self.download_asset_external(self.model_link, "cuda_ctc_decoder/cpu_jit.pt")
        self.bpe_link = f"{self.url_prefix}/resolve/main/data/lang_bpe_500/bpe.model"
        self.bpe_path = self.download_asset_external(self.bpe_link, "cuda_ctc_decoder/bpe.model")
        self.bpe_model = spm.SentencePieceProcessor()
        self.bpe_model.load(self.bpe_path)
        self.tokens = [self.bpe_model.id_to_piece(id) for id in range(self.bpe_model.get_piece_size())]
        torch.random.manual_seed(0)
        self.initialize_asr_model()
        
        self.resampler = T.Resample(self.sample_rate, self.bundle.sample_rate, dtype=torch.float16)
        self.resampler = self.resampler.to(self.torch_device)

        # Audio data storage
        self.voice_sources = []

    def download_asset_external(self, url, key):
        path = Path(torch.hub.get_dir()) / "torchaudio" / Path(key)
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            torch.hub.download_url_to_file(url, path)
        return str(path)

    def initialize_asr_model(self):
        exec('self.bundle = torchaudio.pipelines.%s' % self.am_bundle)
        self.asr_model = self.bundle.get_model().to(self.torch_device)

        self.lm_files = download_pretrained_files("librispeech-4-gram")

        self.beam_search_decoder = ctc_decoder(
            lexicon=os.path.join(get_package_share_directory(self.lexicon_package), self.lexicon_file),
            tokens=self.lm_files.tokens,
            lm=self.lm_files.lm,
            nbest=1,
            beam_size=50,
            lm_weight=self.lm_weight,
            word_score=self.word_score,
            sil_score=self.sil_score
        )

    def asr(self, voice_tensor):

        voice_tensor = voice_tensor.to(self.torch_device)
        self.voice_tensor_resampled = self.resampler(voice_tensor).to(self.torch_device)
        # torch.save(self.voice_tensor_resampled,'voice_data_resampled.pt')

        with torch.inference_mode():
            emission, _ = self.asr_model(self.voice_tensor_resampled.view(1,-1).float())

            # CPU CTC beam search decoder
            beam_search_result = self.beam_search_decoder(emission.cpu())
            scores = [hyp.score for hyp in beam_search_result[0]]
            beam_search_transcript = " ".join(beam_search_result[0][0].words).strip()

            return beam_search_transcript, torch.nn.functional.softmax(torch.Tensor(scores),0)[0].item()

    def audio_data_callback(self, msg):

        self.sources_msg = SpeechAzSources()
        self.sources_msg.header.stamp = self.get_clock().now().to_msg()
        self.sources_msg.header.frame_id = self.microphone_frame_id

        # Check incoming data for voice activity
        if len(msg.sources) > 0:
            new_voice_sources = []
            new_voice_azimuths = []
            new_voice_matches = []

            # For each incoming directional source
            for ii, az_source in enumerate(msg.sources):

                # Put data in a torch tensor
                self.frame = torch.frombuffer(az_source.audio.data,dtype=torch.float16)

                # Check for voice activity 
                self.voice_data = self.vad.forward(self.frame)

                if len(self.voice_data) > self.min_voice_samples:
                    new_voice_sources.append(AudioSource(az_source.azimuth,self.frame))
                    new_voice_azimuths.append(az_source.azimuth)

        # Process voice sources
        jj=0
        if new_voice_sources:

            # Match existing voice_sources with new voice sources
            while self.voice_sources:
                voice = self.voice_sources.pop()

                # Compute distance between existing voice source and incoming voice sources
                az_dist = np.array(new_voice_azimuths) - voice.azimuth

                for ii, az in enumerate(az_dist):
                    if az > np.pi:
                        az_dist[ii] = az - 2*np.pi
                    if az < -np.pi:
                        az_dist[ii] = az + 2*np.pi

                min_idx = np.argmin(np.absolute(az_dist))

                if az_dist[min_idx] < self.src_match_thresh_rad:
                    voice.azimuth = new_voice_sources[min_idx].azimuth
                    voice.append_frame(new_voice_sources[min_idx].frame)
                    new_voice_matches.append(min_idx)
                    new_voice_sources.append(voice)

                else: # If unmatched old source, process, delete, and send message
                    torch.save(voice.frame,'voice_frame_%s.pt' % jj)
                    jj+=1
                    transcript, conf = self.asr(voice.frame)
                    self.source_msg = SpeechAzSource()
                    self.source_msg.azimuth = voice.azimuth
                    self.source_msg.transcript = transcript
                    self.source_msg.confidence = conf
                    self.sources_msg.sources.append(self.source_msg)

            # Create voice sources from any new, unmatched voice sources 
            for ii, source in enumerate(new_voice_sources):
                if ii not in new_voice_matches:
                    self.voice_sources.append(source)

        else: # If unmatched old source, process, delete, and send message
            # Match existing voice_sources with new voice sources
            for jj, voice in enumerate(self.voice_sources):
                torch.save(voice.frame,'voice_frame_%s.pt' % jj)
                jj+=1
                transcript, conf = self.asr(voice.frame)
                self.source_msg = SpeechAzSource()
                self.source_msg.azimuth = voice.azimuth
                self.source_msg.transcript = transcript
                self.source_msg.confidence = conf
                self.sources_msg.sources.append(self.source_msg)

            self.voice_sources = []

        self.source_pub.publish(self.sources_msg)

def main(args=None):
    rclpy.init(args=args)
    speech_node = DirectionalSpeechRecNode()
    rclpy.spin(speech_node)
    
    speech_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
