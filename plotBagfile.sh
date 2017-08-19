#!/usr/bin/gnuplot -persist
set title "Ardrone and Target positions" font ",14" textcolor rgbcolor "royalblue"
#set timefmt "%y/%m/%d"
#set xdata time
set pointsize 1
set datafile separator ","

first=0;
offset=0;
func(x)=(offset=(first==0)?x:offset,first=1,x-offset)
plot "bagfile-_vrpn_client_node_ardrone_pose.csv" (func($1)):($1/10)


set terminal wxt  enhanced title "Bagfile plot " persist raise
#plot "bagfile-_vrpn_client_node_ardrone_pose.csv" using ($0/10):($1/10)
