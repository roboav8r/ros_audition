# hri_audition
A repo for audition and auditory analysis software for my Ph.D. research in human-robot interaction and human-robot teaming.

A lot of this will be application-specific, but if you have specific questions feel free to reach out to me.

LAST UPDATED: March 2023

## Current hardware setup
Azure Kinect microphone array


## TODO / Future Work
General usability
- ROS launch files for sensors and HARK network

Add hardware:
- 16soundsUSB array - use with Kinect array?

Functions I am working on implementing:
- Speaker localization provided by separate LiDAR/Visual ROS tracking node to HARK network
- Beamforming using HARK network
- Automatic Speech Recognition using Kaldi, Julius, or CMUSphinx
- Background noise estimation/feature extraction with HARK
- Background noise classification/scene estimation with Python node
