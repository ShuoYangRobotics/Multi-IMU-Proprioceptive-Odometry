function [sensor_data, param] = get_five_imu_sensor_data_from_rosbag(file_path, param)
% read data bags, generate lists of measurements
% save data to sensor_data struct

% sensor_data = 
% 
%   struct with fields:
% 
%        accel_IMU: [1×1 timeseries]
%         gyro_IMU: [1×1 timeseries]
%        vel_mocap: [1×1 timeseries]
%        pos_mocap: [1×1 timeseries]
%       foot_force: [1×1 timeseries]
%     contact_mode: [1×1 timeseries]
%     joint_torque: [1×1 timeseries]
%        joint_ang: [1×1 timeseries]
%        joint_vel: [1×1 timeseries]


sensor_data = {};

bagselect = rosbag(file_path);
param.data_duration = 999;
duration = param.data_duration;
start_time =bagselect.StartTime;
end_time =bagselect.EndTime;

%% select body IMU data 
bSel2 = select(bagselect,"Time",[start_time start_time + duration],'Topic',param.body_imu_topic);
if bSel2.StartTime > start_time
    start_time = bSel2.StartTime
end
if bSel2.EndTime < end_time
    end_time = bSel2.EndTime
end
param.data_duration = end_time-start_time;
duration = param.data_duration;
accel_IMU = timeseries(bSel2,"LinearAcceleration.X","LinearAcceleration.Y","LinearAcceleration.Z");
accel_IMU.Time = accel_IMU.Time-accel_IMU.Time(1);
accel_IMU.Data = movmean(accel_IMU.Data,5,1);

gyro_IMU = timeseries(bSel2,"AngularVelocity.X","AngularVelocity.Y","AngularVelocity.Z");
gyro_IMU.Time = gyro_IMU.Time-gyro_IMU.Time(1);

gyro_IMU.Data = movmean(gyro_IMU.Data,5,1);

sensor_data.accel_body_IMU = accel_IMU;
sensor_data.gyro_body_IMU = gyro_IMU;

%% select mocap data, process it to get velocity
if param.has_mocap == 1
    bSel = select(bagselect,"Time",[start_time start_time + duration],'Topic',param.mocap_topic);
    if bSel.StartTime > start_time
        start_time = bSel.StartTime
    end
    if bSel.EndTime < end_time
        end_time = bSel.EndTime
    end
    param.data_duration = end_time-start_time;
    duration = param.data_duration;
    orient_mocap = timeseries(bSel,"Pose.Orientation.W", "Pose.Orientation.X","Pose.Orientation.Y","Pose.Orientation.Z");
    orient_mocap.Time = orient_mocap.Time-orient_mocap.Time(1);

    pos_mocap = timeseries(bSel,"Pose.Position.X","Pose.Position.Y","Pose.Position.Z");
    pos_mocap.Time = pos_mocap.Time-pos_mocap.Time(1);
    pos_mocap.Data = pos_mocap.Data-pos_mocap.Data(1,:);
    dt_list = [0.001;pos_mocap.Time(2:end)-pos_mocap.Time(1:end-1)];

    sensor_data.orient_mocap = orient_mocap;
    sensor_data.pos_mocap = pos_mocap;
    %% pos foot
    bSel_foot_fr = select(bagselect,"Time",[start_time start_time + duration],'Topic',param.mocap_FR_topic);


    pos_foot_fr_mocap = timeseries(bSel_foot_fr,"Pose.Position.X","Pose.Position.Y","Pose.Position.Z");

    dt_list = [0.001;pos_foot_fr_mocap.Time(2:end)-pos_foot_fr_mocap.Time(1:end-1)];    
    [b,g] = sgolay(5,25);
    dt = mean(dt_list);
    dx = zeros(length(pos_foot_fr_mocap.Data),3);
    for p = 1:3
      dx(:,p) = conv(pos_foot_fr_mocap.Data(:,p), factorial(1)/(-dt)^1 * g(:,2), 'same');
    end
    vel_foot_fr_mocap = timeseries(dx,pos_foot_fr_mocap.Time,'Name',"foot_fr velocity");

    sensor_data.pos_foot_fr_mocap = pos_foot_fr_mocap;
    sensor_data.vel_foot_fr_mocap = vel_foot_fr_mocap;


end
%% select leg data, must use struct data 
bSel3 = select(bagselect,"Time",[start_time start_time + duration],'Topic',param.joint_foot_topic);
if bSel3.StartTime > start_time
    start_time = bSel3.StartTime
end
if bSel3.EndTime < end_time
    end_time = bSel3.EndTime
end
param.data_duration = end_time-start_time;
duration = param.data_duration;
msgStructs = readMessages(bSel3,'DataFormat','struct');
num_data = size(msgStructs,1);
foot_force_data = zeros(num_data,param.num_leg);
contact_mode_data = zeros(num_data,param.num_leg);
joint_ang_data = zeros(num_data,param.num_dof);
joint_vel_data = zeros(num_data,param.num_dof);
joint_torque_data = zeros(num_data,param.num_dof);
time = zeros(num_data,1);
dt_list = zeros(num_data,1);
start_time = double(msgStructs{1}.Header.Stamp.Sec) + double(msgStructs{1}.Header.Stamp.Nsec)*10^-9;

for i=1:num_data
    time(i) = double(msgStructs{i}.Header.Stamp.Sec) + double(msgStructs{i}.Header.Stamp.Nsec)*10^-9;
    time(i) = time(i) - start_time;
    for j=1:param.num_leg
        contact_mode_data(i,j) = msgStructs{i}.Velocity(12+j);
        foot_force_data(i,j) = msgStructs{i}.Effort(12+j);
    end
    if i >=2 
        dt_list(i) = time(i) - time(i-1);
    end
    for j=1:param.num_dof
        joint_ang_data(i,j) = msgStructs{i}.Position(j);
        joint_torque_data(i,j) = msgStructs{i}.Effort(j);
        joint_vel_data(i,j) = msgStructs{i}.Velocity(j);
    end
end
% process foot_force_data to get contact mode estimation
contact_flag1 = foot_force_data(:,1) > 100;
contact_flag2 = foot_force_data(:,2) > 100;
contact_flag3 = foot_force_data(:,3) > 100;
contact_flag4 = foot_force_data(:,4) > 100;

contact_mode_data = [contact_flag1 contact_flag2 contact_flag3 contact_flag4];

%%
joint_ang_data = movmean(joint_ang_data,5,1);
foot_force = timeseries(foot_force_data,time,'Name',"foot force");
%% by analyzing figure comparing to joint angle and foot IMUs, I think there is a delay in contact sensor
contact_time = max(time - 0.03, 0);
contact_mode = timeseries(contact_mode_data,contact_time,'Name',"contact mode");

joint_ang = timeseries(joint_ang_data,time,'Name',"joint angle");
joint_torque_data = movmean(joint_torque_data,5,1);
joint_torque = timeseries(joint_torque_data,time,'Name',"joint torque");


[b,g] = sgolay(5,11);
dt = mean(dt_list);   
joint_vel_smooth_data = zeros(length(joint_ang.Data),param.num_dof);
for p = 1:param.num_dof
  joint_vel_smooth_data(:,p) = conv(joint_ang_data(:,p), factorial(1)/(-dt)^1 * g(:,2), 'same');
end
joint_vel = timeseries(joint_vel_smooth_data,time,'Name',"joint velocity");


sensor_data.foot_force = foot_force;
sensor_data.contact_mode = contact_mode;
sensor_data.joint_torque = joint_torque;
sensor_data.joint_ang = joint_ang;
sensor_data.joint_vel = joint_vel;

%% get foot IMU data

delay_time_est = 0.023;
bSel_fl = select(bagselect,"Time",[start_time start_time + duration],'Topic',param.fl_imu_topic);
accel_fl_IMU = timeseries(bSel_fl,"LinearAcceleration.X","LinearAcceleration.Y","LinearAcceleration.Z");
accel_fl_IMU.Time = accel_fl_IMU.Time - delay_time_est;
accel_fl_IMU.Time = accel_fl_IMU.Time - accel_fl_IMU.Time(1);

gyro_fl_IMU = timeseries(bSel_fl,"AngularVelocity.X","AngularVelocity.Y","AngularVelocity.Z");
gyro_fl_IMU.Time = gyro_fl_IMU.Time - delay_time_est;
gyro_fl_IMU.Time = gyro_fl_IMU.Time-gyro_fl_IMU.Time(1);
gyro_fl_IMU.Data = gyro_fl_IMU.Data/180*pi; % deg to rad
sensor_data.accel_fl_IMU = accel_fl_IMU;
sensor_data.gyro_fl_IMU = gyro_fl_IMU;

bSel_fr = select(bagselect,"Time",[start_time start_time + duration],'Topic',param.fr_imu_topic);
accel_fr_IMU = timeseries(bSel_fr,"LinearAcceleration.X","LinearAcceleration.Y","LinearAcceleration.Z");
accel_fr_IMU.Time = accel_fr_IMU.Time - delay_time_est;
accel_fr_IMU.Time = accel_fr_IMU.Time-accel_fr_IMU.Time(1);
gyro_fr_IMU = timeseries(bSel_fr,"AngularVelocity.X","AngularVelocity.Y","AngularVelocity.Z");
gyro_fr_IMU.Time = gyro_fr_IMU.Time - delay_time_est;
gyro_fr_IMU.Time = gyro_fr_IMU.Time-gyro_fr_IMU.Time(1);
gyro_fr_IMU.Data = gyro_fr_IMU.Data/180*pi; % deg to rad
sensor_data.accel_fr_IMU = accel_fr_IMU;
sensor_data.gyro_fr_IMU = gyro_fr_IMU;

bSel_rl = select(bagselect,"Time",[start_time start_time + duration],'Topic',param.rl_imu_topic);
accel_rl_IMU = timeseries(bSel_rl,"LinearAcceleration.X","LinearAcceleration.Y","LinearAcceleration.Z");
accel_rl_IMU.Time = accel_rl_IMU.Time - delay_time_est;
accel_rl_IMU.Time = accel_rl_IMU.Time-accel_rl_IMU.Time(1);
gyro_rl_IMU = timeseries(bSel_rl,"AngularVelocity.X","AngularVelocity.Y","AngularVelocity.Z");
gyro_rl_IMU.Time = gyro_rl_IMU.Time - delay_time_est;
gyro_rl_IMU.Time = gyro_rl_IMU.Time-gyro_rl_IMU.Time(1);
gyro_rl_IMU.Data = gyro_rl_IMU.Data/180*pi; % deg to rad
sensor_data.accel_rl_IMU = accel_rl_IMU;
sensor_data.gyro_rl_IMU = gyro_rl_IMU;


bSel_rr = select(bagselect,"Time",[start_time start_time + duration],'Topic',param.rr_imu_topic);
accel_rr_IMU = timeseries(bSel_rr,"LinearAcceleration.X","LinearAcceleration.Y","LinearAcceleration.Z");
accel_rr_IMU.Time = accel_rr_IMU.Time - delay_time_est;
accel_rr_IMU.Time = accel_rr_IMU.Time-accel_rr_IMU.Time(1);
gyro_rr_IMU = timeseries(bSel_rr,"AngularVelocity.X","AngularVelocity.Y","AngularVelocity.Z");
gyro_rr_IMU.Time = gyro_rr_IMU.Time - delay_time_est;
gyro_rr_IMU.Time = gyro_rr_IMU.Time-gyro_rr_IMU.Time(1);
gyro_rr_IMU.Data = gyro_rr_IMU.Data/180*pi; % deg to rad
sensor_data.accel_rr_IMU = accel_rr_IMU;
sensor_data.gyro_rr_IMU = gyro_rr_IMU;

% save some final time related variable
param.data_start_time = start_time;
param.data_end_time = end_time;



end
