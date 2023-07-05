#!/usr/bin/env python3

import torch
import torchaudio
import rospy
from std_msgs.msg import String
from hri_msgs.msg import AudioFeatures
import time

class FrameCacher:
    """Cache the end of the last frame, and append the hop length / shift to the end of it.

    Args:
        total_length (int): The total size of main segment (samples).
            If the incoming shift data is shorter, then the data is padded to return the segment.
        shift_length (int): The size of the incoming hop/shift data to be appended.
    """

    def __init__(self, total_length: int, hop_length: int, n_channels: int, cudadev):
        self.total_length = total_length
        self.hop_length = hop_length
        self.total_segment = torch.zeros([total_length, n_channels])
        self.hop_segment = torch.zeros([hop_length, n_channels])
        self.total_segment.to(cudadev)
        self.hop_segment.to(cudadev)
        # TODO - move tensor to GPU

    def __call__(self, in_chunk: torch.Tensor):
        # self.hop_segment = in_chunk[-self.context_length :]
        self.hop_segment = in_chunk

        # Roll the existing segment back by the hop length
        self.total_segment = torch.roll(self.total_segment, -self.hop_length, 0)

        # Assign new hop data at the end of frame (in place of older, rolled hop data)
        self.total_segment[-self.hop_length:,:] = in_chunk

        return self.total_segment

class TorchAudioStream:
    def __init__(self, dev=0, hop=160, fs=16000):
        self.device = dev  # Set the default value for the device
        self.hop = hop
        self.sample_rate = fs

        # Create FFMPEG streamer object
        self.streamer = torchaudio.io.StreamReader("hw:" + str(self.device), "alsa")

        # Create a source stream
        self.streamer.add_audio_stream(
            frames_per_chunk=self.hop,
            filter_desc=f"aresample={self.sample_rate},anull,aformat=sample_fmts=fltp"
        )

        self.stream_iterator = self.streamer.stream()

        print(self.streamer.num_src_streams)
        print(self.streamer.get_out_stream_info(0))

    def hop_chunk_ready(self):
        if self.streamer.is_buffer_ready():
            print("Hop chunk ready")
        return self.streamer.is_buffer_ready()
    
    def get_hop_chunk(self):
        # return next(self.stream_iterator)[0]
        return self.streamer.pop_chunks()
        
    #     # Create a publisher
    #     self.publisher = rospy.Publisher('output_topic', String, queue_size=10)


    # def publish_message(self, message):
    #     # Publish the message
    #     self.publisher.publish(message)

def main():
    # Initialize the ROS node
    rospy.init_node('audio_feat_ext_node')

    # Create the publisher object
    message = String()
    pub = rospy.Publisher('output_topic', String, queue_size=1)

    # Load parameters from the parameter server
    dev_num = rospy.get_param("alsadevnum")
    n_channels = rospy.get_param("n_channels")
    frame_size = rospy.get_param("frame_size")
    hop_size = rospy.get_param("hop_size")
    f_sample = rospy.get_param("sample_rate")

    # Create an instance of the TorchAudioStream class
    audio_stream = TorchAudioStream(dev_num, hop_size, f_sample)

    # CUDA
    cuda0 = torch.device('cuda:0')

    # Create the frame cache class, initialize 
    frame_cache = FrameCacher(frame_size, hop_size, n_channels, cuda0)
    frame_tensor = torch.zeros([frame_size,n_channels])
    hop_tensor = torch.zeros([hop_size,n_channels])
    frame_tensor.to(cuda0)
    hop_tensor.to(cuda0)

    # Main loop
    rate = rospy.Rate(f_sample/hop_size)
    print(0.8*f_sample/hop_size)

    audio_stream.streamer.fill_buffer()

    while not rospy.is_shutdown():
        # Try to get the latest hop chunk

        # Get the hop chunk
        tic = time.perf_counter()
        audio_stream.streamer.fill_buffer(timeout=-1,backoff=1)
        print(f"Buffer took {time.perf_counter() - tic:0.4f} seconds to fill")
        hop_tensor = audio_stream.get_hop_chunk()[0][:,:]
        # print(hop_tensor.shape)

        # Put in frame tensor
        frame_tensor = frame_cache(hop_tensor)

        # Extract features from the cached frame

        # Example usage: publish a message
        
        message.data = 'Got hop_tensor with shape: ' + str(hop_tensor.shape) + ' and frame_tensor with shape ' + str(frame_tensor.shape)
        pub.publish(message)

        # rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
