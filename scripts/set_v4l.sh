#!/bin/bash

video_id=60
 
v_dev=/dev/video${video_id}

if ! ls "$v_dev" 2> /dev/null
then
  if lsmod | grep -wq v4l2loopback
  then
    sudo modprobe -r v4l2loopback
  fi
  sudo modprobe v4l2loopback video_nr=$video_id card_label="theta_x" exclusive_caps=1
fi
