function [sensor_data, param] = get_sensor_data_from_dataset(file_path, param)
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
param.data_duration = floor(bagselect.EndTime-bagselect.StartTime)-6;
param.data_duration = 60;
duration = param.data_duration;
start_time =bagselect.StartTime;

%% select IMU data 
bSel2 = select(bagselect,"Time",[start_time start_time + duration+0.4],'Topic','/hardware_a1/imu');
accel_IMU = timeseries(bSel2,"LinearAcceleration.X","LinearAcceleration.Y","LinearAcceleration.Z");
accel_IMU.Time = accel_IMU.Time-accel_IMU.Time(1);
accel_IMU.Data = movmean(accel_IMU.Data,40,1);

gyro_IMU = timeseries(bSel2,"AngularVelocity.X","AngularVelocity.Y","AngularVelocity.Z");
gyro_IMU.Time = gyro_IMU.Time-gyro_IMU.Time(1);

gyro_IMU.Data = movmean(gyro_IMU.Data,40,1);

sensor_data.accel_IMU = accel_IMU;
sensor_data.gyro_IMU = gyro_IMU;
%% select mocap data, process it to get velocity
if (param.has_mocap)
bSel = select(bagselect,"Time",[start_time start_time + accel_IMU.Time(end)-0.5],'Topic','/mocap_node/Robot_1/pose');
orient_mocap = timeseries(bSel,"Pose.Orientation.W", "Pose.Orientation.X","Pose.Orientation.Y","Pose.Orientation.Z");
orient_mocap.Time = orient_mocap.Time-orient_mocap.Time(1);

orientatation_gt_euler_data = zeros(size(orient_mocap.Data,1),3);
for i=1:size(orient_mocap.Data,1)
%     if i > 1 
%         qdiff = orient_mocap.Data(i,:)-orient_mocap.Data(i-1,:);
%         qsum  = orient_mocap.Data(i,:)+orient_mocap.Data(i-1,:);
%         if norm(qdiff) < norm(qsum)
%             orientatation_gt_euler_data(i,:) = quat_to_euler(-orient_mocap.Data(i,:));
%         else
% 
%             orientatation_gt_euler_data(i,:) = quat_to_euler(orient_mocap.Data(i,:));
%         end
%     else
%             orientatation_gt_euler_data(i,:) = quat_to_euler(orient_mocap.Data(i,:));
%     end
    orientatation_gt_euler_data(i,:) = rot_to_euler(quat_to_rot(orient_mocap.Data(i,:)));
end
orientatation_gt_euler = timeseries(orientatation_gt_euler_data, orient_mocap.Time,'Name',"euler gt");

pos_mocap = timeseries(bSel,"Pose.Position.X","Pose.Position.Y","Pose.Position.Z");
pos_mocap.Time = pos_mocap.Time-pos_mocap.Time(1);
pos_mocap.Data = pos_mocap.Data-pos_mocap.Data(1,:);
dt_list = [0.001;pos_mocap.Time(2:end)-pos_mocap.Time(1:end-1)];

%%
[b,g] = sgolay(5,25);
dt = mean(dt_list);
dx = zeros(length(pos_mocap.Data),3);
for p = 1:3
  dx(:,p) = conv(pos_mocap.Data(:,p), factorial(1)/(-dt)^1 * g(:,2), 'same');
end

% this velocity uses the IMU marker as root so
% p_b = p_r + R_er*p_br
% v_b = v_r + R_er*w*p_br
p_rb = [0.2293;0;0.095];
for idx = 2:size(pos_mocap.Time,1)
    t = pos_mocap.Time(idx);    
    R_er = quat2rotm(orient_mocap.Data(idx,:));
    tmp_idxs = find(gyro_IMU.Time-t>=0);
    % interpolate to get state(mocap pos and orientation)
    imu_t1 = gyro_IMU.Time(tmp_idxs(1)-1);
    imu_t2 = gyro_IMU.Time(tmp_idxs(1));
    alpha = (t-imu_t1)/(imu_t2-imu_t1);
    wi = (1-alpha)*gyro_IMU.Data(tmp_idxs(1)-1,:) + alpha*gyro_IMU.Data(tmp_idxs(1),:);

    dx(idx,:) = dx(idx,:) - (R_er*cross(wi,p_rb)')';
end

vel_mocap = timeseries(dx,pos_mocap.Time,'Name',"mocap velocity");

% pos_mocap = intergrate_vel_ts(vel_mocap);


sensor_data.orient_mocap = orient_mocap;
sensor_data.orientatation_gt_euler = orientatation_gt_euler;
sensor_data.vel_mocap = vel_mocap;
sensor_data.pos_mocap = pos_mocap;
end
%% select leg data, must use struct data 
bSel3 = select(bagselect,"Time",[start_time start_time + duration],'Topic','/hardware_a1/joint_foot');
msgStructs = readMessages(bSel3,'DataFormat','struct');
num_data = size(msgStructs,1);
foot_force_data = zeros(num_data,param.num_leg);
contact_mode_data = zeros(num_data,param.num_leg);
%contact_mode_data = zeros(num_data,param.num_leg);
joint_ang_data = zeros(num_data,param.num_dof);
joint_vel_data = zeros(num_data,param.num_dof);
joint_torque_data = zeros(num_data,param.num_dof);
time = zeros(num_data,1);
dt = zeros(num_data,1);
start_time = double(msgStructs{1}.Header.Stamp.Sec) + double(msgStructs{1}.Header.Stamp.Nsec)*10^-9;

for i=1:num_data
    time(i) = double(msgStructs{i}.Header.Stamp.Sec) + double(msgStructs{i}.Header.Stamp.Nsec)*10^-9;
    time(i) = time(i) - start_time;
    for j=1:param.num_leg
        contact_mode_data(i,j) = msgStructs{i}.Velocity(12+j);
        if (msgStructs{i}.Effort(12+j) > 0)
            foot_force_data(i,j) = msgStructs{i}.Effort(12+j);
        else
            foot_force_data(i,j) = 0;
        end
    end
    if i >=2 
        dt(i) = time(i) - time(i-1);
    end
    for j=1:param.num_dof
        joint_ang_data(i,j) = msgStructs{i}.Position(j);
        joint_torque_data(i,j) = msgStructs{i}.Effort(j);
%         joint_vel_data(i,j) = msgStructs{i}.Velocity(j);
%         if i >=2 
%             joint_vel_data(i,j) = (joint_ang_data(i,j)-joint_ang_data(i-1,j))/dt(i);
%         end
    end
end
%%
joint_ang_data = movmean(joint_ang_data,15,1);
foot_force = timeseries(foot_force_data,time,'Name',"foot force");
if (param.has_contact_flag)
controller_contact_mode = timeseries(contact_mode_data,time,'Name',"controller contact mode");
end
joint_ang = timeseries(joint_ang_data,time,'Name',"joint angle");
joint_torque_data = movmean(joint_torque_data,15,1);
joint_torque = timeseries(joint_torque_data,time,'Name',"joint torque");


[b,g] = sgolay(5,11);
dt = 0.002;   %  HARDWARE_FEEDBACK_FREQUENCY in A1_Ctrl
joint_vel_smooth_data = zeros(length(joint_ang.Data),param.num_dof);
for p = 1:param.num_dof
  joint_vel_smooth_data(:,p) = conv(joint_ang_data(:,p), factorial(1)/(-dt)^1 * g(:,2), 'same');
end
joint_vel_smooth_data(1:100,:) = 0;
joint_vel_smooth_data(end-100:end,:) = 0;
joint_vel = timeseries(joint_vel_smooth_data,time,'Name',"joint velocity");


sensor_data.foot_force = foot_force;

sensor_data.joint_torque = joint_torque;
sensor_data.joint_ang = joint_ang;
sensor_data.joint_vel = joint_vel;
sensor_data.controller_contact_mode = controller_contact_mode;

end
