%% init sensor data
run ../kinematics_init_lc;  % add libraries generate kinematics as matlab functions
run ../param_init;          % get param as global variable
addpath("../rot_lib")
addpath("../")
addpath("../common_func/")
addpath("../casadi/")
warning('off')
% read rosbag, modify the path for different dataset
disp('read rosbag');

file_path = ['/home/shuoyang/rosbag/20230215_aaron_lab/',...
    '_2023-02-15-15-01-56.bag'] 

param.fl_imu_topic = '/WT901_49_Data';
param.fr_imu_topic = '/WT901_48_Data';
param.rl_imu_topic = '/WT901_50_Data';
param.rr_imu_topic = '/WT901_47_Data';
param.body_imu_topic = '/unitree_hardware/imu';
param.mocap_topic = '/mocap_node/Go1_body/pose';
param.mocap_FR_topic = '/mocap_node/Go1_FR/pose';
param.joint_foot_topic = '/unitree_hardware/joint_foot';


%% read rosbag data
% %generate sensor_data, modify param
sensor_data_process;

%% set some filter parameters for mipo and sipo
param.init_cov = 0.1;
param.init_bias_cov = 1e-4;
param.init_body_height = 0.3;

param.data_start_idx = 200;

% noise parameters
param.proc_n_pos = 0.0005;
param.proc_n_vel_xy = 0.005;
param.proc_n_vel_z = 0.005;
param.proc_n_ang = 1e-7;
param.proc_n_foot_pos = 1e-4;
param.proc_n_foot_vel = 2;   % only for mipo
param.proc_n_ba = 1e-4;
param.proc_n_bg = 1e-5;
param.proc_n_foot1_ba = 1e-4;    % only for mipo
param.proc_n_foot2_ba = 1e-4;    % only for mipo
param.proc_n_foot3_ba = 1e-4;    % only for mipo
param.proc_n_foot4_ba = 1e-4;    % only for mipo

param.ctrl_n_acc = 1e-1;
param.ctrl_n_gyro = 1e-3;
param.ctrl_n_foot1_acc = 1e-1;
param.ctrl_n_foot2_acc = 1e-1;
param.ctrl_n_foot3_acc = 1e-1;
param.ctrl_n_foot4_acc = 1e-1;

param.meas_n_fk_pos = 0.001;
param.meas_n_fk_vel = 0.01;
param.meas_n_foot_height = 0.001;
param.meas_n_zero_vel = 0.01;      % for foot imu


%% run baseline sipo filter 
tic 
[sipo_state_list] = run_sipo(re_sensor_data, param);
sipo_time = toc
%% run mipo filter 
param.mipo_use_foot_ang_contact_model = 1; % 0 means use zero velocity model
param.mipo_use_md_test_flag = 1;           % 0 means use contact flag
tic 
[mipo_state_list] = run_mipo(re_sensor_data, param);
mipo_time = toc
%%
plot_start=1;
plot_end = size(sipo_state_list(end,:),2);
figure(69);clf;
if param.has_mocap == 1
    plot3(re_sensor_data.pos_mocap.Data(:,1),re_sensor_data.pos_mocap.Data(:,2),re_sensor_data.pos_mocap.Data(:,3), 'LineWidth',1.3)
    hold on;
end
plot3(movmean(sipo_state_list(1,plot_start:plot_end),5,1), ...
      movmean(sipo_state_list(2,plot_start:plot_end),5,1), ...
      movmean(sipo_state_list(3,plot_start:plot_end),5,1), 'LineWidth',1.3);
hold on;
plot3(mipo_state_list(1,plot_start:plot_end),mipo_state_list(2,plot_start:plot_end),mipo_state_list(3,plot_start:plot_end), 'LineWidth',1.3);

legend("Ground truth", "SIPO","MIPO", "Location","northeast")
axis equal

xlabel("X Position (m)")
ylabel("Y Position (m)")
view(-0,90)