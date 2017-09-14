imu =  csvread('bagfile-_ardrone_imu.csv',1);
navdata = csvread('bagfile-_ardrone_navdata.csv',1);
cmd_vel_PBVS = csvread('bagfile-_cmd_vel_PBVS.csv',1);
path = csvread('bagfile-_path.csv',1);
vrpn_client_node_ardrone = csvread('bagfile-_vrpn_client_node_ardrone_pose.csv',1);
odometry = csvread('bagfile-_ardrone_odometry.csv',1);
land = csvread('bagfile-_ardrone_land.csv',1);
des_pos = csvread('bagfile-_des_pos.csv',1);
takeoff = csvread('bagfile-_ardrone_takeoff.csv',1);
des_vel = csvread('bagfile-_des_vel.csv',1);
src_cmd = csvread('bagfile-_src_cmd.csv',1);
cmd_vel = csvread('bagfile-_cmd_vel.csv',1);
mag = csvread('bagfile-_ardrone_mag.csv',1);
tf = csvread('bagfile-_tf.csv',1);
opti_data = csvread('bagfile-_opti_data.csv',1);

imumin = imu(1,1)
navdatamin = navdata(1,1)
cmd_vel_PBVSmin = cmd_vel_PBVS(1,1)
pathmin = path(1,1)
vrpn_client_node_ardronemin = vrpn_client_node_ardrone(1,1)
odometrymin = odometry(1,1)
landmin = land(1,1)
des_posmin = des_pos(1,1);
takeoffmin = takeoff(1,1);
des_velmin = des_vel(1,1);
src_cmdmin = src_cmd(1,1);
cmd_velmin = cmd_vel(1,1);
magmin = mag(1,1);

min_time = min(imumin,navdatamin,cmd_vel_PBVSmin,pathmin,vrpn_client_node_ardronemin,\
odometrymin,landmin,des_posmin,takeoffmin,des_velmin,src_cmdmin,cmd_velmin,magmin);





