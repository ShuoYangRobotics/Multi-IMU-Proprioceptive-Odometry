param.has_mocap = 1;  % mocap
[sensor_data, param] = get_five_imu_sensor_data_from_rosbag(file_path, param);

param.has_vilo = 0;  % has no vilo

%resample sensor data
re_sensor_data = resample_sensor_data(sensor_data, 0.005, param);
