function [state_list] = run_sipo(re_sensor_data, param)


total_steps = size(re_sensor_data.accel_body_IMU.Time,1);
total_start_idx = param.data_start_idx;
total_end_idx =  total_steps-1;
N = total_end_idx - total_start_idx + 1; 
total_start_time = re_sensor_data.accel_body_IMU.Time(total_start_idx);
disp('run SIPO');
% http://www.roboticsproceedings.org/rss08/p03.pdf
% init kf
[sipo_conf,param] = sipo_conf_init(param);
% init foot pos
foot_pos_list = zeros(3*param.num_leg,1);
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
    foot_pos_list((i-1)*3+1:(i-1)*3+3) = R_er*p_rf +init_pos;
end

x0 = [
    init_pos;
    zeros(3,1);
    init_euler;
    foot_pos_list;
    zeros(3,1);
    zeros(3,1);
    0];
x_list = zeros(sipo_conf.state_size, N+1);
x_list(:,1) = x0;

P0 = param.init_cov*eye(sipo_conf.state_size);
P0(22:27,22:27) = param.init_bias_cov*eye(6);
cov_list = zeros(sipo_conf.state_size,sipo_conf.state_size, N+1);
cov_list(:,:,1) = P0;

for idx=total_start_idx:total_end_idx
    k = idx-total_start_idx+1;
    dt = re_sensor_data.gyro_body_IMU.Time(idx+1) - re_sensor_data.gyro_body_IMU.Time(idx);
    uk = [re_sensor_data.gyro_body_IMU.Data(idx,:)';re_sensor_data.accel_body_IMU.Data(idx,:)';dt];
    uk1 = [re_sensor_data.gyro_body_IMU.Data(idx+1,:)';re_sensor_data.accel_body_IMU.Data(idx+1,:)';dt]; 

    % run kalman filter to get velocity and position
    x01 = full(sipo_conf.f(x_list(:,k) , uk, uk1, dt));
    F = full(sipo_conf.df(x_list(:,k) , uk, uk1, dt));
    B = full(sipo_conf.db(x_list(:,k) , uk, uk1, dt));

    sipo_conf.Q1 = diag([param.proc_n_pos* dt *ones(2,1); % pos x y
                         param.proc_n_pos* dt *ones(1,1);   % pos z
                         param.proc_n_vel_xy * dt *ones(1,1); % vel x 
                         param.proc_n_vel_xy * dt *ones(1,1); % vel y
                         param.proc_n_vel_z  * dt *ones(1,1);  % vel z 
                         param.proc_n_ang *  dt *ones(3,1);
                         param.proc_n_foot_pos *  dt *ones(12,1);  % foot pos
                         param.proc_n_ba *  dt *ones(3,1);    % acc bias random walk
                         param.proc_n_bg *  dt *ones(3,1);    % gyro bias random walk
                         0;]);  
  
    % control noise 
    sipo_conf.Q2 = diag([param.ctrl_n_acc * dt *ones(3,1);   % acc noise should be large around contact
                         param.ctrl_n_gyro * dt *ones(3,1);   % gyro noise
                         0]);  
           
    ck = double(re_sensor_data.contact_mode.Data(idx,:) ); 

    num_meas = param.sipo_meas_per_leg;
    for i = 1:param.num_leg
        % foot position process noise
        sipo_conf.Q1(9+(i-1)*3+1:9+(i-1)*3+3,9+(i-1)*3+1:9+(i-1)*3+3)= ...
            (1 + (1 - ck(i)) * 1e5)*param.proc_n_foot_pos *dt*eye(3);

        sipo_conf.R((i-1)*num_meas+1:(i-1)*num_meas+3,(i-1)*num_meas+1:(i-1)*num_meas+3) = ...
            (1 + (1 - ck(i)) * 1e5)*param.meas_n_fk_pos *dt*eye(3);
        sipo_conf.R((i-1)*num_meas+4:(i-1)*num_meas+6,(i-1)*num_meas+4:(i-1)*num_meas+6) = ...
            (1 + (1 - ck(i)) * 1e5)*param.meas_n_fk_vel *dt*eye(3);
        sipo_conf.R((i-1)*num_meas+7,(i-1)*num_meas+7) = ...
            (1 + (1 - ck(i)) * 1e5)*param.meas_n_foot_height *dt;
    end

    P01 = F*cov_list(:,:,k)*F' + sipo_conf.Q1 + B*sipo_conf.Q2*B';
 
    hat_wk = re_sensor_data.gyro_body_IMU.Data(idx,:)';
    hat_phik = re_sensor_data.joint_ang.Data(idx,:)';
    hat_dphik = re_sensor_data.joint_vel.Data(idx,:)';

    hat_yawk = re_sensor_data.orient_mocap_euler.Data(idx,3);

    y = full(sipo_conf.r(x01, hat_wk, hat_phik, hat_dphik, hat_yawk));
    H = full(sipo_conf.dr(x01, hat_wk, hat_phik, hat_dphik, hat_yawk));

    S = H*P01*H' + sipo_conf.R;

    mask = ones(sipo_conf.meas_size,1);
    mask = logical(mask);
    update  = P01 * H(mask,:)' * (S(mask,mask)\y(mask));

    x_list(:,k+1) = x01 - update; 
    cov_list(:,:,k+1) = ( eye(sipo_conf.state_size)-P01*H(mask,:)'*(S(mask,mask)\H(mask,:)))*P01;


    cov_list(:,:,k+1) = (cov_list(:,:,k+1) + cov_list(:,:,k+1)')/2;

end

state_list = x_list;






end