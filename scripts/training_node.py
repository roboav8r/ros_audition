#!/usr/bin/env python3

import pandas as pd

import rospy
import rosbag
import rospkg
import os

from audio_common_msgs.msg import AudioInfo, AudioDataStamped
from hri_msgs.msg import AudioFeatures

from hri_audition.srv import TrainAudioModels, TrainAudioModelsResponse

class AudioTrainNode:

    def __init__(self):

        # Initialize member variables
        self.feat_msg = AudioFeatures()
        self.rospack = rospkg.RosPack()
        self.pkg_path = self.rospack.get_path('hri_audition')
        self.wav_path = self.pkg_path + '/data/wav/'
        self.feat_path = self.pkg_path + '/data/features/'

        # Set up ROS subs and pubs
        self.wav_pub = rospy.Publisher('/audio/audio_stamped', AudioDataStamped, queue_size=10)
        self.feat_sub = rospy.Subscriber('/audio/audio_features', AudioFeatures, self.feat_cb)

        # Set up server
        self.train_server = rospy.Service("train_audio_models", TrainAudioModels, self.handle_train_audio_models)

    def feat_cb(self, msg):
        self.feat_msg = msg

    def generate_feat_bags(self):
        print('Converting wave .bags to feature .bags')
        # Iterate through wave filepath
        for root, dirs,files in os.walk(self.wav_path,topdown=True):
            for file in files:
                filepath = os.path.join(root,file)
                
                # Get type label of datafile
                type_dir = os.path.dirname(filepath)
                type_str = os.path.split(type_dir)[-1]
                
                # Get base filename and class label of datafile
                base = os.path.splitext(file)[0]
                label = base.split('_')[0]
                
                # Check if feature directory and output bag exist. if not, create them
                if os.path.exists(self.feat_path)==False:
                    os.mkdir(self.feat_path)
                
                if os.path.exists(self.feat_path + '/' + type_str + '/')==False:
                    os.mkdir(self.feat_path + '/' + type_str + '/')
                
                if os.path.exists(self.feat_path + '/' + type_str + '/' + base +'.bag')==False:
                    with rosbag.Bag(self.feat_path + '/' + type_str + '/' + file,'w') as feat_bag:

                        # Load bag and iterate through messages
                        wav_bag = rosbag.Bag(filepath)

                        for tpc,msg,t in wav_bag.read_messages():
                            if (msg._type == 'audio_common_msgs/AudioDataStamped'):
                                self.wav_pub.publish(msg)
                                rospy.sleep(0.001) 
                                feat_bag.write('/audio_features',self.feat_msg) 

    def feat_bags_to_df(self):
        
        print('Forming feature .bag data into DataFrame')

        # Initialize 
        cols = ['type','name','zcr','rms','pitch','hnr',
        'mfcc_1','mfcc_2','mfcc_3','mfcc_4','mfcc_5','mfcc_6',
        'mfcc_7','mfcc_8','mfcc_9','mfcc_10','mfcc_11','mfcc_12']
        self.feat_df = pd.DataFrame(columns=cols)
        df_idx = 0

        # Traverse file structure and convert bag data to dataframe data
        for root, dirs, files in os.walk(self.feat_path,topdown=True):
            for file in files:
                filepath = os.path.join(root,file)
                
                # Get type label of datafile
                type_dir = os.path.dirname(filepath)
                type_str = os.path.split(type_dir)[-1]
                
                # Get class label of datafile
                base = os.path.splitext(file)[0]
                base = base.split('_')[0]
                
                # Get extension of datafile - if it's a bag, process and generate dataframes
                ext = os.path.splitext(file)[-1]

                if ext=='.bag':
                    # Check if feature directory and output bag exist. if not, create them
                    if os.path.exists(self.feat_path)==False:
                        os.mkdir(self.feat_path)

                    if os.path.exists(self.feat_path + '/' + type_str + '/')==False:
                        os.mkdir(self.feat_path + '/' + type_str + '/')

                    # Load bag and iterate through messages
                    feat_bag = rosbag.Bag(filepath)
                    
                    for tpc,msg,t in feat_bag.read_messages():

                        if (msg._type == 'hri_msgs/AudioFeatures' and msg.ZCR != 0.0):
                            self.feat_df.loc[df_idx,'type'] = type_str
                            self.feat_df.loc[df_idx,'name'] = base
                            self.feat_df.loc[df_idx,'zcr'] = msg.ZCR
                            self.feat_df.loc[df_idx,'rms'] = msg.RMS
                            self.feat_df.loc[df_idx,'pitch'] = msg.pitch
                            self.feat_df.loc[df_idx,'hnr'] = msg.HNR
                            self.feat_df.loc[df_idx,'mfcc_1'] = msg.MFCC[0]
                            self.feat_df.loc[df_idx,'mfcc_2'] = msg.MFCC[1]
                            self.feat_df.loc[df_idx,'mfcc_3'] = msg.MFCC[2]
                            self.feat_df.loc[df_idx,'mfcc_4'] = msg.MFCC[3]
                            self.feat_df.loc[df_idx,'mfcc_5'] = msg.MFCC[4]
                            self.feat_df.loc[df_idx,'mfcc_6'] = msg.MFCC[5]
                            self.feat_df.loc[df_idx,'mfcc_7'] = msg.MFCC[6]
                            self.feat_df.loc[df_idx,'mfcc_8'] = msg.MFCC[7]
                            self.feat_df.loc[df_idx,'mfcc_9'] = msg.MFCC[8]
                            self.feat_df.loc[df_idx,'mfcc_10'] = msg.MFCC[9]
                            self.feat_df.loc[df_idx,'mfcc_11'] = msg.MFCC[10]
                            self.feat_df.loc[df_idx,'mfcc_12'] = msg.MFCC[11]

                            df_idx+=1

    def save_df_to_csvs(self):
        print('Saving DataFrames to .csv')
        for root, dirs, files in os.walk(self.feat_path,topdown=True):
            for file in files:
                filepath = os.path.join(root,file)
                
                # Get type label of datafile
                type_dir = os.path.dirname(filepath)
                type_str = os.path.split(type_dir)[-1]

                # Get class label of datafile
                base = os.path.splitext(file)[0]
                base = base.split('_')[0]
                
                # Check if feature directory and output bag exist. if not, create them
                if os.path.exists(self.feat_path)==False:
                    os.mkdir(self.feat_path)
                
                if os.path.exists(self.feat_path + '/' + type_str + '/')==False:
                    os.mkdir(self.feat_path + '/' + type_str + '/')
                    
                self.feat_df.loc[(self.feat_df['type']==type_str) & (self.feat_df['name']==base)].to_csv(root + '/' + base + '.csv')  
                
    def handle_train_audio_models(self, req):
        print('Training Audio Models')
        self.generate_feat_bags()

        self.feat_bags_to_df()

        self.save_df_to_csvs()

        return TrainAudioModelsResponse()

    def run(self):
        rospy.spin()



if __name__ == '__main__':
    rospy.init_node('audio_trainer_node')
    node = AudioTrainNode()
    node.run()