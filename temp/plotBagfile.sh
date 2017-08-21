#!/bin/bash

gnuplot -persist <<-EOFMarker
    set autoscale
    set yrange [-0.5:0.5]
    set datafile separator ","
    set key autotitle columnhead
    set grid
    plot "bagfile-_cmd_vel.csv" using (\$1):2 with linespoints
    first=GPVAL_DATA_X_MIN
    plot "bagfile-_cmd_vel.csv" using ((\$1-first)/10**9):2, "bagfile-_img_data.csv" using ((\$1-first)/10**9):((\$3-180)/360)
EOFMarker

gnuplot -persist <<-EOFMarker
    set autoscale
    set datafile separator ","
    set yrange [-0.5:0.5]
    set key autotitle columnhead
    set grid
    plot "bagfile-_cmd_vel.csv" using (\$1):2 with linespoints
    first=GPVAL_DATA_X_MIN
    plot "bagfile-_cmd_vel.csv" using ((\$1-first)/10**9):3, "bagfile-_img_data.csv" using ((\$1-first)/10**9):((\$2-360)/720)
EOFMarker
# rest of script, after gnuplot exits
gnuplot -persist <<-EOFMarker
    set autoscale
    set datafile separator ","
    set key autotitle columnhead
    set grid
    plot "bagfile-_vrpn_client_node_ardrone_pose.csv" using (\$1):5 with linespoints
    first=GPVAL_DATA_X_MIN
    plot "bagfile-_vrpn_client_node_ardrone_pose.csv" using ((\$1-first)/10**9):7, "bagfile-_vrpn_client_node_target_pose.csv" using ((\$1-first)/10**9):7
EOFMarker
gnuplot -persist <<-EOFMarker
    set autoscale
    set datafile separator ","
    set yrange [-0.5:0.5]
    set key autotitle columnhead
    set grid
    plot "bagfile-_vrpn_client_node_ardrone_pose.csv" using (\$1):2 with linespoints
    first=GPVAL_DATA_X_MIN
    plot "bagfile-_vrpn_client_node_ardrone_pose.csv" using ((\$1-first)/10**9):5, "bagfile-_vrpn_client_node_target_pose.csv" using ((\$1-first)/10**9):5
EOFMarker
