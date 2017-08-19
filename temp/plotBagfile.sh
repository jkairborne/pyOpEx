#!/bin/bash
minval=0    # the result of some (omitted) calculation
maxval=10   # ditto
gnuplot -persist <<-EOFMarker
    print "-"
    set autoscale
    set datafile separator ","
    set key autotitle columnhead
    plot "bagfile-_ardrone_navdata.csv" using (\$1):5 with linespoints
    first=GPVAL_DATA_X_MIN
    plot 'bagfile-_ardrone_navdata.csv' using ((\$1-first)/10**9):5 with linespoints
    print(first)

EOFMarker

# rest of script, after gnuplot exits
