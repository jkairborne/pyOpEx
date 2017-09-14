#!/bin/bash

gnuplot -persist <<-EOFMarker
    set autoscale
	set datafile separator ","
    set key autotitle columnhead
    set grid
    plot "bagfile-_cmd_vel.csv" using (\$1):2 with linespoints
    first=GPVAL_DATA_X_MIN
    plot "bagfile-_visual_params.csv" using ((\$1-first)/10**9):2 title "x-est", "bagfile-_visual_params.csv" using ((\$1-first)/10**9):3 title "y-est" 
EOFMarker


gnuplot -persist <<-EOFMarker
    set autoscale
	set datafile separator ","
    set key autotitle columnhead
    set grid
    plot "bagfile-_cmd_vel.csv" using (\$1):2 with linespoints
    first=GPVAL_DATA_X_MIN
    plot "bagfile-_visual_params.csv" using ((\$1-first)/10**9):2 title "x-est", "bagfile-_vrpn_client_node_ardrone_pose.csv" using ((\$1-first)/10**9):5  title "Ardrone x position","bagfile-_vrpn_client_node_roomba_pose.csv" using ((\$1-first)/10**9):5  title "Roomba x position"
EOFMarker

gnuplot -persist <<-EOFMarker
    set autoscale
	set datafile separator ","
    set key autotitle columnhead
    set grid
    plot "bagfile-_cmd_vel.csv" using (\$1):2 with linespoints
    first=GPVAL_DATA_X_MIN
    plot "bagfile-_visual_params.csv" using ((\$1-first)/10**9):3 title "y-est", "bagfile-_vrpn_client_node_ardrone_pose.csv" using ((\$1-first)/10**9):7  title "Ardrone y position","bagfile-_vrpn_client_node_roomba_pose.csv" using ((\$1-first)/10**9):7  title "Roomba y position"
EOFMarker


# rest of script, after gnuplot exit
