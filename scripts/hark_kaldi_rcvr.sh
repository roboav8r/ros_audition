#!/bin/bash

# gnome-terminal --tab -e 'bash -c "python3 scripts/HarkDataStreamReceiver.py"' --tab -e 'bash -c "sleep 0.2 && ../config/kinectHarkNetwork.n"' --tab -e 'bash -c "sleep 1.5 && python3 sendHarkParam.py"'
gnome-terminal --window --title Hark Data Receiver