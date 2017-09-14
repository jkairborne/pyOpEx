#!/bin/bash

gnuplot -persist <<-EOFMarker
    set autoscale
    set datafile separator ","
    set key autotitle columnhead
    set grid
    plot "bagfile-_cmd_vel.csv" using (\$1):2 with linespoints
    first=GPVAL_DATA_X_MIN
    a=1
    plot "bagfile-_cmd_vel.csv" using ((\$1-first)/10**9):2 title "cmd vel_x", "bagfile-_vrpn_client_node_ardrone_pose.csv" using ((\$1-first)/10**9):5  title "Optitrack x position","bagfile-_vrpn_client_node_roomba_pose.csv" using ((\$1-first)/10**9):5 title "Roomba x position" 
EOFMarker

gnuplot -persist <<-EOFMarker
    set autoscale
    set datafile separator ","
    set key autotitle columnhead
    set grid
    plot "bagfile-_cmd_vel.csv" using (\$1):2 with linespoints
    first=GPVAL_DATA_X_MIN
    plot "bagfile-_cmd_vel.csv" using ((\$1-first)/10**9):(-\$3) title "cmd vel_y", "bagfile-_vrpn_client_node_ardrone_pose.csv" using ((\$1-first)/10**9):7 title "Optitrack ardrone y position","bagfile-_vrpn_client_node_roomba_pose.csv" using ((\$1-first)/10**9):7 title "Roomba y position" 
EOFMarker

gnuplot -persist <<-EOFMarker
    set datafile separator ","
    set key autotitle columnhead
    set grid
    plot "bagfile-_cmd_vel.csv" using (\$1):2 with linespoints
    first=GPVAL_DATA_X_MIN
	set autoscale
    plot "bagfile-_cmd_vel.csv" using ((\$1-first)/10**9):4 title "cmd vel_z", "bagfile-_vrpn_client_node_ardrone_pose.csv" using ((\$1-first)/10**9):6 title "Optitrack ardrone z position","bagfile-_vrpn_client_node_roomba_pose.csv" using ((\$1-first)/10**9):6 title "Roomba z position" 
EOFMarker


# rest of script, after gnuplot exit
