#!/bin/bash
gnuplot -persist <<-EOFMarker
    set autoscale
    set datafile separator ","
    set key autotitle columnhead
    plot "bag1" using (\$1):2 with linespoints
    first=GPVAL_DATA_X_MIN
    plot "bag2" using (\$1):3 with linespoints
    second=GPVAL_DATA_X_MIN
    plot "bag1" using ((\$1-first)/10**9):2, "bag2" using ((\$1-second)/10**9):3
EOFMarker
