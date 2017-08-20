#!/bin/bash

gnuplot -persist <<-EOFMarker
    print "-"
    set autoscale
    set datafile separator " "
    set key autotitle columnhead
    #print(first)
    set style rect fc lt -1 fs solid 0.15 noborder

	myarr=($(awk '$2==1 {print $1}' data.dat))

	# Now access elements of an array (change "1" to whatever you want)
	echo ${myarr[1]}
	echo ${#myarr[@]}
	#echo $((${#myarr[@]}/2))

	# Or loop through every element in the array${#ArrayName[@]}
	#for i in "$((${#myarr[@]}/2))"   
	#do
	 #  :
	#   echo $i
	  #set obj rect from ${myarr[i]}, graph 0 to 
	#done

	#set obj rect from 1, graph 0 to 2, graph 1	
	#set obj rect from 3, graph 0 to 4, graph 1
	plot "data.dat" using 1:3
    
EOFMarker
