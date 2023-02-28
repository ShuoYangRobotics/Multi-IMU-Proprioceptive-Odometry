function [state_list] = run_mipo(re_sensor_data, param)


total_steps = size(re_sensor_data.accel_body_IMU.Time,1);
total_start_idx = param.data_start_idx;
total_end_idx =  total_steps-1;
N = total_end_idx - total_start_idx + 1; 
total_start_time = re_sensor_data.accel_body_IMU.Time(total_start_idx);
disp('run MIPO');
% init kf
[mipo_conf,param] = mipo_conf_init(param);
% init foot pos and vel
foot_pos_vel_list = zeros(6*param.num_leg,1);
if param.has_mocap == 1
    init_pos = resample(re_sensor_data.pos_mocap, total_start_time).Data';
else
    init_pos = [0;0;param.init_body_height];
end
init_euler = zeros(3,1);
R_er = euler_to_rot(init_euler);
phik = resample(re_sensor_data.joint_ang, total_start_time).Data';
for i = 1:param.num_leg
    angle = phik((i-1)*3+1:(i-1)*3+3);
    p_rf = autoFunc_fk_pf_pos(angle,param.rho_opt_true(:,i),param.rho_fix(:,i));
    foot_pos_vel_list((i-1)*6+1:(i-1)*6+3) = R_er*p_rf +init_pos;
end

x0 = [
    init_pos;
    zeros(3,1);
    init_euler;
    foot_pos_vel_list;
    zeros(3,1);
    zeros(3,1);
    zeros(3,1);
    zeros(3,1);
    zeros(3,1);
    zeros(3,1);
    0];
x_list = zeros(mipo_conf.state_size, N+1);
x_list(:,1) = x0;


P0 = param.init_cov*eye(mipo_conf.state_size);
P0(34:51,34:51) = param.init_bias_cov*eye(18);
cov_list = zeros(mipo_conf.state_size,mipo_conf.state_size, N+1);
cov_list(:,:,1) = P0;

for idx=total_start_idx:total_end_idx
    k = idx-total_start_idx+1;
    dt = re_sensor_data.gyro_body_IMU.Time(idx+1) - re_sensor_data.gyro_body_IMU.Time(idx);
    % convert foot acc from foot frame to body frame using FK rotation
    joint_angs = re_sensor_data.joint_ang.Data(idx,:)';
    accel_IMUs = [
                    re_sensor_data.accel_fl_IMU.Data(idx,:)';
                    re_sensor_data.accel_fr_IMU.Data(idx,:)';
                    re_sensor_data.accel_rl_IMU.Data(idx,:)';
                    re_sensor_data.accel_rr_IMU.Data(idx,:)';
                 ];
    gyro_IMUs = [
                    re_sensor_data.gyro_fl_IMU.Data(idx,:)';
                    re_sensor_data.gyro_fr_IMU.Data(idx,:)';
                    re_sensor_data.gyro_rl_IMU.Data(idx,:)';
                    re_sensor_data.gyro_rr_IMU.Data(idx,:)';
                 ];
    accel_IMU_bs = zeros(3*param.num_leg,1);    % from IMU fram to robot body frame
    gyro_IMU_bs = zeros(3*param.num_leg,1);
    for leg_id=1:param.num_leg
        leg_joint_angs = joint_angs((leg_id-1)*3+1:(leg_id-1)*3+3);
        R_bf = autoFunc_fk_pf_rot(leg_joint_angs,param.rho_opt_true(:,leg_id),param.rho_fix(:,leg_id));
        accel_IMU_bs((leg_id-1)*3+1:(leg_id-1)*3+3) = R_bf*param.R_fm_list{leg_id}*accel_IMUs((leg_id-1)*3+1:(leg_id-1)*3+3);
        gyro_IMU_bs((leg_id-1)*3+1:(leg_id-1)*3+3) = R_bf*param.R_fm_list{leg_id}*gyro_IMUs((leg_id-1)*3+1:(leg_id-1)*3+3);
    end
    uk = [
            re_sensor_data.gyro_body_IMU.Data(idx,:)';
            re_sensor_data.accel_body_IMU.Data(idx,:)';
            accel_IMU_bs;
            dt];

    joint_angs = re_sensor_data.joint_ang.Data(idx+1,:)';
    accel_IMUs = [
                    re_sensor_data.accel_fl_IMU.Data(idx+1,:)';
                    re_sensor_data.accel_fr_IMU.Data(idx+1,:)';
                    re_sensor_data.accel_rl_IMU.Data(idx+1,:)';
                    re_sensor_data.accel_rr_IMU.Data(idx+1,:)';
                 ];
    gyro_IMUs = [
                    re_sensor_data.gyro_fl_IMU.Data(idx+1,:)';
                    re_sensor_data.gyro_fr_IMU.Data(idx+1,:)';
                    re_sensor_data.gyro_rl_IMU.Data(idx+1,:)';
                    re_sensor_data.gyro_rr_IMU.Data(idx+1,:)';
                 ];
    accel_IMU_bs1 = zeros(3*param.num_leg,1);
    gyro_IMU_bs1 = zeros(3*param.num_leg,1);
    for leg_id=1:param.num_leg
        leg_joint_angs = joint_angs((leg_id-1)*3+1:(leg_id-1)*3+3);
        R_bf = autoFunc_fk_pf_rot(leg_joint_angs,param.rho_opt_true(:,leg_id),param.rho_fix(:,leg_id));
        accel_IMU_bs1((leg_id-1)*3+1:(leg_id-1)*3+3) = R_bf*param.R_fm_list{leg_id}*accel_IMUs((leg_id-1)*3+1:(leg_id-1)*3+3);
        gyro_IMU_bs1((leg_id-1)*3+1:(leg_id-1)*3+3) = R_bf*param.R_fm_list{leg_id}*gyro_IMUs((leg_id-1)*3+1:(leg_id-1)*3+3);
    end
    uk1 = [
            re_sensor_data.gyro_body_IMU.Data(idx+1,:)';
            re_sensor_data.accel_body_IMU.Data(idx+1,:)';
            accel_IMU_bs1;
            dt];

    x01 = full(mipo_conf.f(x_list(:,k) , uk, uk1, dt));
    F = full(mipo_conf.df(x_list(:,k) , uk, uk1, dt));
    B = full(mipo_conf.db(x_list(:,k) , uk, uk1, dt));

    mipo_conf.Q1 = diag([param.proc_n_pos* dt *ones(2,1); % pos x y
                         param.proc_n_pos* dt *ones(1,1);   % pos z
                         param.proc_n_vel_xy * dt *ones(1,1); % vel x 
                         param.proc_n_vel_xy * dt *ones(1,1); % vel y
                         param.proc_n_vel_z  * dt *ones(1,1);  % vel z 
                         param.proc_n_ang *  dt *ones(3,1);
                        repmat(...
                            [param.proc_n_foot_pos * dt*ones(2,1);  % foot1 pos x y
                             param.proc_n_foot_pos * dt*ones(1,1);  % foot1 pos z
                             param.proc_n_foot_vel  * dt*ones(2,1);  % foot1 vel  x y         
                             param.proc_n_foot_vel * dt*ones(1,1)],4,1);  % foot1 vel z
                         param.proc_n_ba *  dt *ones(3,1);    % acc bias random walk
                         param.proc_n_bg *  dt *ones(3,1);    % gyro bias random walk
                         param.proc_n_foot1_ba *  dt *ones(3,1);    % foot1 acc bias random walk
                         param.proc_n_foot2_ba *  dt *ones(3,1);    % foot1 acc bias random walk
                         param.proc_n_foot3_ba *  dt *ones(3,1);    % foot1 acc bias random walk
                         param.proc_n_foot4_ba *  dt *ones(3,1);    % foot1 acc bias random walk
                         0;]);  
  
    % control noise 
    mipo_conf.Q2 = diag([param.ctrl_n_acc * dt *ones(3,1);   % acc noise should be large around contact
                         param.ctrl_n_gyro * dt *ones(3,1);   % gyro noise
                         param.ctrl_n_foot1_acc * dt *ones(3,1);   % gyro noise
                         param.ctrl_n_foot2_acc * dt *ones(3,1);   % gyro noise
                         param.ctrl_n_foot3_acc * dt *ones(3,1);   % gyro noise
                         param.ctrl_n_foot4_acc * dt *ones(3,1);   % gyro noise
                         0]);  
           
    ck = double(re_sensor_data.contact_mode.Data(idx,:) ); 

    num_meas = param.mipo_meas_per_leg;
    for i = 1:param.num_leg
        if (param.mipo_use_md_test_flag == 0)
            mipo_conf.R((i-1)*num_meas+7:(i-1)*num_meas+9,(i-1)*num_meas+7:(i-1)*num_meas+9) = ...
                (1 + (1 - ck(i)) * 1e5)*param.meas_n_zero_vel *dt*eye(3);
            mipo_conf.R((i-1)*num_meas+10,(i-1)*num_meas+10) = ...
                (1 + (1 - ck(i)) * 1e5)*param.meas_n_foot_height *dt;
        else
            mipo_conf.R((i-1)*num_meas+7:(i-1)*num_meas+9,(i-1)*num_meas+7:(i-1)*num_meas+9) = ...
                param.meas_n_zero_vel *eye(3);
            mipo_conf.R((i-1)*num_meas+10,(i-1)*num_meas+10) = ...
                param.meas_n_foot_height;
        end
    end

    P01 = F*cov_list(:,:,k)*F' + mipo_conf.Q1 + B*mipo_conf.Q2*B';
 
    hat_wk = re_sensor_data.gyro_body_IMU.Data(idx,:)';
    hat_phik = re_sensor_data.joint_ang.Data(idx,:)';
    hat_dphik = re_sensor_data.joint_vel.Data(idx,:)';

    hat_yawk = re_sensor_data.orient_mocap_euler.Data(idx,3);

    y = full(mipo_conf.r(x01, hat_wk, hat_phik, hat_dphik, hat_yawk, gyro_IMU_bs));
    H = full(mipo_conf.dr(x01, hat_wk, hat_phik, hat_dphik, hat_yawk, gyro_IMU_bs));

    S = H*P01*H' + mipo_conf.R;

    
    mask = ones(mipo_conf.meas_size,1);
    if (param.mipo_use_md_test_flag == 1)
        for i = 1:param.num_leg
            seg_mes = y((i-1)*num_meas+7:(i-1)*num_meas+9);
            seg_S = S((i-1)*num_meas+7:(i-1)*num_meas+9,(i-1)*num_meas+7:(i-1)*num_meas+9);
            MD = sqrt(seg_mes'*inv(seg_S)*seg_mes);
            if MD > 1
                mask((i-1)*num_meas+7:(i-1)*num_meas+9) = zeros(3,1);
            end
        end
    end
    mask = logical(mask);
    update  = P01 * H(mask,:)' * (S(mask,mask)\y(mask));

    x_list(:,k+1) = x01 - update; 
    cov_list(:,:,k+1) = ( eye(mipo_conf.state_size)-P01*H(mask,:)'*(S(mask,mask)\H(mask,:)))*P01;


    cov_list(:,:,k+1) = (cov_list(:,:,k+1) + cov_list(:,:,k+1)')/2;
end

state_list = x_list;






end