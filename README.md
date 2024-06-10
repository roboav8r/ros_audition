# ros_audition
A repo for audition and auditory analysis in ROS2, used for my Ph.D. research in human-robot interaction and human-robot teaming.

A lot of this will be application-specific, but if you have specific questions or would like to collaborate, feel free to reach out to me.

# Setup
## Finding audio device parameters
To find your audio input device parameters, install pulseaudio and run:
```
pacmd list-sources
```
Find your device's index and note the following lines:
```
sample spec: s16le 1ch 48000Hz
device.string = "hw:3"
device.description = "Webcam C270 Mono"
```

Then, create a `<device>_config.yaml file`, using the provided examples as a starting point.

# Future work
## Improvements
- Refactor launch file input arguments
- Account for 16/32 signed/unsigned datatypes in audio_acq's audio frame
- Handle the audio_common_msgs dependency and build error