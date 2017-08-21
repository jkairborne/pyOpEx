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
