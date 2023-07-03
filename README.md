# hri_audition (ROS branch)
A repo for audition and auditory analysis software for my Ph.D. research in human-robot interaction and human-robot teaming.

A lot of this will be application-specific, but if you have specific questions feel free to reach out to me.

LAST UPDATED: May 2023

## Prerequisites
This assumes you are using Ubuntu 20.04 with ROS Noetic installed.

### Package Dependencies
Torch, torchaudio

## Current hardware setup
Azure Kinect microphone array

## Usage

### One-time setup
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

### Speech recognition


## Other useful info
### Getting sound card sampling rate info
```
arecord -f dat -r 48000 -D plughw:2,0 -d 5 test.wav # Verify card supports 48000 kHz sample rate
```

## TODO / Future Work

TODO add package deps to package.xml and CMakeLists and rosdep

Add hardware:
- 16soundsUSB array - use with Kinect array simultaneously?
