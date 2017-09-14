#!/bin/bash
sed -i -e 's/_base_link//g' *
sed -i -e 's/_base_footprint//g' *
sed -i -e 's/_base_frontcam//g' *
sed -i -e 's/_base_bottomcam//g' *
sed -i -e 's/,world//g' *
sed -i -e 's/,ardrone//g' *
sed -i -e 's/,roomba//g' *
sed -i -e 's/,target//g' *
sed -i -e 's/,odom//g' *

