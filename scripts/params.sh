#!/bin/bash
cd $1;
sleep 2.0;
gnome-terminal --title="Parameter Reconfiguration" -- python3 scripts/sendHarkParam.py