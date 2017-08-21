#!/bin/bash

gnuplot -persist <<-EOFMarker
    set autoscale
    set datafile separator ","
    set key autotitle columnhead
    plot "bagfile-_cmd_vel.csv" using (\$1):2 with linespoints
    first=GPVAL_DATA_X_MIN
    plot "bagfile-_img_data.csv" using (\$1):3 with linespoints
    second=GPVAL_DATA_X_MIN
    plot "bagfile-_cmd_vel.csv" using ((\$1-first)/10**9):2, "bagfile-_img_data.csv" using ((\$1-second)/10**9):((\$3-180)/180)
EOFMarker

# rest of script, after gnuplot exits
