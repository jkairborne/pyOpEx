#!/bin/bash

gnuplot -persist <<-EOFMarker
    set autoscale
    set datafile separator ","
    set key autotitle columnhead
    set grid
    plot "bagfile-_ardrone_navdata.csv" using (\$1):5 with linespoints
    first=GPVAL_DATA_X_MIN
    plot "bagfile-_ardrone_navdata.csv" using ((\$1-first)/10**9):16, "bagfile-_cmd_vel.csv" using ((\$1-first)/10**9):2
EOFMarker
gnuplot -persist <<-EOFMarker
    set autoscale
    set datafile separator ","
    set yrange [-0.5:0.5]
    set key autotitle columnhead
    set grid
    plot "bagfile-_ardrone_navdata.csv" using (\$1):2 with linespoints
    first=GPVAL_DATA_X_MIN
    plot "bagfile-_ardrone_navdata.csv" using ((\$1-first)/10**9):15, "bagfile-_cmd_vel.csv" using ((\$1-first)/10**9):3
EOFMarker
