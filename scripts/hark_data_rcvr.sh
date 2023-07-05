#!/bin/bash
cd $1;
sleep 0.5;
gnome-terminal --title="HARK Data Receiver" -- python3 scripts/HarkDataStreamReceiver.py 