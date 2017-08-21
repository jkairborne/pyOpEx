#!/bin/bash

gnuplot -persist <<-EOFMarker
    set autoscale
    set key autotitle columnhead
	first=0;
	offset=0;
	func(x)=(offset=(first==0)\?x:offset,first=1,x-offset)
	#firstoffset=func(\$1)
	plot 'tst.csv' using (func(\$1)):2
   # plot "bagfile-_cmd_vel.csv" using ((\$1-first)/10**9):2, "bagfile-_img_data.csv" using ((\$1-second)/10**9):3#((\$3-180)/180)
EOFMarker

# rest of script, after gnuplot exits
