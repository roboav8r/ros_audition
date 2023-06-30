# hri_audition
A repo for audition and auditory analysis software for my Ph.D. research in human-robot interaction and human-robot teaming.

A lot of this will be application-specific, but if you have specific questions feel free to reach out to me.

LAST UPDATED: May 2023

## Prerequisites

### openSMILE
To use openSMILE feature extraction, build from source. In your home directory:
```
cd ~
sudo apt-get install portaudio19-dev
sudo apt-get install libopencv*
sudo apt-get install gnuplot-nox
git clone https://github.com/roboav8r/opensmile.git
cd opensmile/
```
Back at the command line, run the build script and install:
```
./build.sh
cd build
sudo make install
```
Note: This is designed to use OpenCV version 4. If you are getting errors, see this issue: https://github.com/audeering/opensmile/issues/17#issuecomment-847654469 


### Kaldi
Download Kaldi speech recognition network from https://hark.jp/download/samples/ (use IROS 2018 practice2)
Extract

move kaldi_conf to hri_audition

TODO write scripts for these ^^

### Package Dependencies
sudo apt install kaldidecoder-hark

TODO add these to package.xml and CMakeLists

## Current hardware setup
Azure Kinect microphone array


## Usage

### openSMILE signal acquisition
```
SMILExtract -C config/mono_microphone.conf # Run a mono microphone array, e.g. from webcam
SMILExtract -C config/16soundsUSB.conf # Run 16soundsUSB device
SMILExtract -C config/16soundsUSB.conf -listDevices 1 # If you get device errors, check device indices
SMILExtract -C config/16soundsUSB.conf -device 11
```

### openSMILE spectrogram generation
When you run `16soundsUSB.conf`, it will automatically generate `spectrogram.csv` in the directory where you ran SMILExtract (e.g. ./hri_audition/)
To generate the spectrogram.png from the .csv:
```
cd ~/opensmile/scripts/gnuplot # This should be my fork located at https://github.com/roboav8r/opensmile
cp ~/mmhri_ws/src/hri_audition/spectrogram.csv ../..
./plot4spectrogram.plt 
```

### Speech recognition
python3 src/hri_audition/scripts/Kaldi.py


### Dummy ROS message to send for HARK sound source locations:
```
rostopic pub -r 10 /speaker_source_loc hark_msgs/HarkSource "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
count: 1
exist_src_num: 1
src:
- {id: 0, power: 1.0, x: 0.5, y: 0.0, z: 0.0, azimuth: 0.0, elevation: 0.0}" 

```

## Other useful info
### Getting sound card sampling rate info
```
arecord -f dat -r 48000 -D plughw:2,0 -d 5 test.wav # Verify card supports 48000 kHz sample rate
```

## TODO / Future Work

Cleanup & remove older files

General usability

Add hardware:
- 16soundsUSB array - use with Kinect array?

Functions I am working on implementing:
- Speaker localization provided by separate LiDAR/Visual ROS tracking node to HARK network
- Automatic Speech Recognition using Kaldi, Julius, or CMUSphinx
- Background noise classification/scene estimation with Python node

Debugging/improves
- Accept channel_selector as input argument (e.g. <Vector<int> 0 1 2 ... >). Currently, HARK can cast input args as int:ARG#, string:ARG#, or float:ARG#, but not object:ARG# datatype (see MAIN_LOOP in the HARK network and HARK cookbook section 7.1)