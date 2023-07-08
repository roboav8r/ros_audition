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

# Usage

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
Next, create a .yaml file in `/config/`. I want to use the Kinect at **card 2**, hence my `kinect_torch.yaml` file has a line `alsadevnum: 2` to specify the device parameter.

Update the launch file to load this file as appropriate.

## Running the speaker recognition demo
TODO
## Running the sound scene classification demo
TODO

# References
Real-time Speech and Music Classification by Large Audio Feature Space Extraction by Florian Eyben.
https://link.springer.com/book/10.1007/978-3-319-27299-3

# Future Work
## Functional additions
- Add package deps to package.xml, add rosdep instructions
- Add Azure Kinect and 16soundsUSB array config files
- Launch multiple audio acquisition use with Kinect array simultaneously (multiple array acquisition)?

## Improvements
CAPTURE
- add args to config/launch files

feat_ext_node
- add option to publish node (default = false)
- make n_bytes in buf_to_float a param
