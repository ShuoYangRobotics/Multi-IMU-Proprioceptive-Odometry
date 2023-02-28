function re_sensor_data = resample_sensor_data(sensor_data, dt, param)

end_time_list = [sensor_data.accel_body_IMU.Time(end);
                 sensor_data.gyro_body_IMU.Time(end);
                 sensor_data.accel_fl_IMU.Time(end);
                 sensor_data.gyro_fl_IMU.Time(end);
                 sensor_data.accel_fr_IMU.Time(end);
                 sensor_data.gyro_fr_IMU.Time(end);
                 sensor_data.accel_rl_IMU.Time(end);
                 sensor_data.gyro_rl_IMU.Time(end);
                 sensor_data.accel_rr_IMU.Time(end);
                 sensor_data.gyro_rr_IMU.Time(end);
                 sensor_data.contact_mode.Time(end)];

min_end_time = min(end_time_list);

re_sensor_data = {};

re_sensor_data.accel_body_IMU = resample(sensor_data.accel_body_IMU, 0:dt:min_end_time);
re_sensor_data.gyro_body_IMU  = resample(sensor_data.gyro_body_IMU, 0:dt:min_end_time);
re_sensor_data.accel_fl_IMU   = resample(sensor_data.accel_fl_IMU, 0:dt:min_end_time);
re_sensor_data.gyro_fl_IMU    = resample(sensor_data.gyro_fl_IMU, 0:dt:min_end_time);
re_sensor_data.accel_fr_IMU   = resample(sensor_data.accel_fr_IMU, 0:dt:min_end_time);
re_sensor_data.gyro_fr_IMU    = resample(sensor_data.gyro_fr_IMU, 0:dt:min_end_time);
re_sensor_data.accel_rl_IMU   = resample(sensor_data.accel_rl_IMU, 0:dt:min_end_time);
re_sensor_data.gyro_rl_IMU    = resample(sensor_data.gyro_rl_IMU, 0:dt:min_end_time);
re_sensor_data.accel_rr_IMU   = resample(sensor_data.accel_rr_IMU, 0:dt:min_end_time);
re_sensor_data.gyro_rr_IMU    = resample(sensor_data.gyro_rr_IMU, 0:dt:min_end_time);
re_sensor_data.foot_force     = resample(sensor_data.foot_force, 0:dt:min_end_time);
re_sensor_data.contact_mode   = resample(sensor_data.contact_mode, 0:dt:min_end_time);
re_sensor_data.joint_torque   = resample(sensor_data.joint_torque, 0:dt:min_end_time);
re_sensor_data.joint_ang      = resample(sensor_data.joint_ang, 0:dt:min_end_time);
re_sensor_data.joint_vel      = resample(sensor_data.joint_vel, 0:dt:min_end_time);

if param.has_mocap == 1
    re_sensor_data.orient_mocap   = resample(sensor_data.orient_mocap, 0:dt:min_end_time);
    re_sensor_data.pos_mocap      = resample(sensor_data.pos_mocap, 0:dt:min_end_time);

    % convert re_sensor_data.orient_mocap to euler angle
    euler_angs_data = zeros(size(re_sensor_data.orient_mocap.Data,1),3);
    for i=1:size(re_sensor_data.orient_mocap.Data,1)
        euler_angs_data(i,:) = quat_to_euler(re_sensor_data.orient_mocap.Data(i,:));
        % make yaw angle continous
        if i>=2
            if euler_angs_data(i-1,3)>pi/5*4 && euler_angs_data(i,3)<euler_angs_data(i-1,3)-pi
                euler_angs_data(i,3) = euler_angs_data(i,3) + 2*pi;
            elseif euler_angs_data(i-1,3)<-pi/5*4 && euler_angs_data(i,3)>euler_angs_data(i-1,3)+pi
                euler_angs_data(i,3) = euler_angs_data(i,3) - 2*pi;
            end
            
        end
    end
    euler_angs_data(:,3) = movmean(euler_angs_data(:,3),10,1);
    re_sensor_data.orient_mocap_euler = timeseries(euler_angs_data,re_sensor_data.orient_mocap.Time,'Name',"euler angle true");
end

end