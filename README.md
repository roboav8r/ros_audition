# hri_audition (ROS1 branch)
A repo for audition and auditory analysis software for my Ph.D. research in human-robot interaction and human-robot teaming.

This repo currently includes progress towards a demo which can perform speaker recognition and sound scene recognition using monochannel audio.

For questions, issues, and suggestions feel free to reach out to me or raise an issue/feature request!

LAST UPDATED: July 2023

# Prerequisites
This assumes you are using Ubuntu 20.04 with ROS Noetic installed.

## Package Dependencies
Torch, torchaudio

## Hardware setup
For the speaker recognition/sound scene classification demo, all you need is a monochannel webcam microphone.

# Installation
Make a catkin workspace, clone the repo, install dependencies, and build. At a terminal:
```
cd ~
mkdir -p my_audio_ws/src
cd my_audio_ws/src/
git clone https://github.com/roboav8r/hri_audition.git
cd ~/my_audio_ws
rosdep install --from-paths src --ignore-src -r -y 
python3 -m pip install -r requirements.txt
catkin build
source devel/setup.bash
```

## One-time setup
First, find the device you want to use for audition. At a terminal, `arecord -l`
```
**** List of CAPTURE Hardware Devices ****
card 0: PCH [HDA Intel PCH], device 0: ALC295 Analog [ALC295 Analog]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 2: Array [Azure Kinect Microphone Array], device 0: USB Audio [USB Audio]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
```
Next, update the .yaml file in `/config/`. For example, I want to use the Kinect at **card 2**, hence my `kinect_example.yaml` file has a line `alsadevnum: 1` to specify the device parameter.

Update the launch file to load this file as appropriate.

# Usage
## Running the speaker recognition demo
TODO
## Running the sound scene classification demo
TODO

# References
Real-time Speech and Music Classification by Large Audio Feature Space Extraction by Florian Eyben.
https://link.springer.com/book/10.1007/978-3-319-27299-3

# Future Work

## Functional additions
- Add multichannel support in acquisition / feat extraction
- Add A16soundsUSB array config files
- Launch multiple audio acquisition use with Kinect array simultaneously (multiple array acquisition)?

## Due Diligence
Validate feature sets - develop test cases (e.g. generated signal with known features)

## Improvements to existing functions
feat_ext_node
- add option to publish frame (default = false)
- make n_bytes in buf_to_float a param based on audio_info