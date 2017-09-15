
imu =  csvread('bagfile-_ardrone_imu.csv',1);
imumin = imu(1,1)
mt = imumin

navdata = csvread('bagfile-_ardrone_navdata.csv',1);

% cmd_vel_PBVS = csvread('bagfile-_cmd_vel_PBVS.csv',1);
% cmd_vel_PBVSmin = cmd_vel_PBVS(1,1)

% path = csvread('bagfile-_path.csv',1);
% pathmin = path(1,1)

vrpn_client_node_ardrone = csvread('bagfile-_vrpn_client_node_ardrone_pose.csv',1);
vrpn_client_node_ardronemin = vrpn_client_node_ardrone(1,1)
vrpn_client_node_ardrone(:,1) = (vrpn_client_node_ardrone(:,1)-mt)/10^9

vrpn_client_node_roomba = csvread('bagfile-_vrpn_client_node_roomba_pose.csv',1);
% vrpn_client_node_roombamin = vrpn_client_node_ardrone(1,1)

vrpn_client_node_roomba(:,1) = (vrpn_client_node_ardrone(:,1)-mt)/10^9

odometry = csvread('bagfile-_ardrone_odometry.csv',1);
% odometrymin = odometry(1,1)
% land = csvread('bagfile-_ardrone_land.csv',1);
% des_pos = csvread('bagfile-_des_pos.csv',1);
% takeoff = csvread('bagfile-_ardrone_takeoff.csv',1);
% des_vel = csvread('bagfile-_des_vel.csv',1);
% src_cmd = csvread('bagfile-_src_cmd.csv',1);
% 
% landmin = land(1,1)
% des_posmin = des_pos(1,1);
% takeoffmin = takeoff(1,1);
% des_velmin = des_vel(1,1);
% src_cmdmin = src_cmd(1,1);

cmd_vel = csvread('bagfile-_cmd_vel.csv',1);
% cmd_velmin = cmd_vel(1,1);
cmd_vel = (cmd_vel(:,1)-mt)/10^9

mag = csvread('bagfile-_ardrone_mag.csv',1);
% magmin = mag(1,1);

% tf = csvread('bagfile-_tf.csv',1);
% opti_data = csvread('bagfile-_opti_data.csv',1);
visual_params = csvread('bagfile-_visual_params.csv',1);
visual_paramsmin = visual_params(1,1);
visual_params(:,1) = (visual_params(:,1)-mt)/10^9

% min_time = min(imumin,navdatamin,cmd_vel_PBVSmin,pathmin,vrpn_client_node_ardronemin,\
% odometrymin,landmin,des_posmin,takeoffmin,des_velmin,src_cmdmin,cmd_velmin,magmin);

figure()
hold on;
plot(visual_params(:,1),visual_params(:,2))
plot(visual_params(:,1),visual_params(:,3))

