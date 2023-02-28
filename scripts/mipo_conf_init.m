function [kf_conf,param] = mipo_conf_init(param)
import casadi.*

% still use casadi for calculting jacobians

kf_conf = {};

kf_conf.state_size = 52;   
% state x 
%   - position          (1:3)
%   - velocity          (4:6)
%   - euler angle       (7:9)
%   - foot1 pos         (10:12)
%   - foot1 vel         (13:15)
%   - foot2 pos         (16:18)
%   - foot2 vel         (19:21)
%   - foot3 pos         (22:24)
%   - foot3 vel         (25:27)
%   - foot4 pos         (28:30)
%   - foot4 vel         (31:33)
%   - body acc bias     (34:36)
%   - body gyro bias    (37:39)
%   - foot1 acc bias    (40:42)
%   - foot2 acc bias    (43:45)
%   - foot3 acc bias    (46:48)
%   - foot4 acc bias    (49:51)
%   - time  tk          (52)

param.mipo_meas_per_leg = 11;  % 3 3 3 1 
kf_conf.meas_size = param.mipo_meas_per_leg * param.num_leg + 1; % yaw
param.mipo_meas_size = kf_conf.meas_size;

kf_conf.ctrl_size = 19;  
% control u 
%   - w      (1:3)      body IMU angular veolocity
%   - a       (4:6)     body IMU acceleration
%   - a1      (7:9)     foot 1 IMU acceleration (already in body frame)
%   - a2      (10:12)   foot 2 IMU acceleration (already in body frame)
%   - a3      (13:15)   foot 3 IMU acceleration (already in body frame)
%   - a4      (16:18)   foot 4 IMU acceleration (already in body frame)
%   - hk       (19)


% process noise
kf_conf.Q1 = diag([1e-4*ones(3,1); 
           1e-2*ones(3,1); 
           1e-6*ones(3,1);
           repmat(...
           [1e-4*ones(2,1);  % foot1 pos x y
            1e-4*ones(1,1);  % foot1 pos z
            1e-4*ones(2,1);  % foot1 vel  x y
            1e-4*ones(1,1)],4,1);  % foot1 vel z
           1e-4*ones(3,1);    % acc bias random walk
           1e-4*ones(3,1);    % gyro bias random walk
           repmat(...
           [1e-4*ones(3,1)],4,1);  % foot acc bias
           0;]);                  % do not use tk in KF

% control noise 
kf_conf.Q2 = diag([1e-4*ones(3,1);   % BODY acc noise
                   1e-4*ones(3,1);   % gyro noise
                   repmat(...
           [1e-4*ones(3,1)],4,1);    % foot acc noise
                   0]);

kf_conf.R = 1e-2*eye(kf_conf.meas_size);

% get process jacobians
s_xk = casadi.MX.sym('X_k', kf_conf.state_size);
s_uk = casadi.MX.sym('Uk', kf_conf.ctrl_size);
s_uk1 = casadi.MX.sym('Uk1', kf_conf.ctrl_size);
s_dt = casadi.MX.sym('dt', 1);
s_f = dyn_rk4(s_xk , s_uk, s_uk1, s_dt, @mipo_process_dyn); % EKF process
s_F = jacobian(s_f, s_xk);
s_B = jacobian(s_f, s_uk);
kf_conf.f = Function('process',{s_xk, s_uk, s_uk1, s_dt},{s_f});
kf_conf.df = Function('process_jac',{s_xk, s_uk, s_uk1, s_dt},{s_F});
kf_conf.db = Function('control_jac',{s_xk, s_uk, s_uk1, s_dt},{s_B});

% get measurement jacobians
s_phik = casadi.MX.sym('phik', 12);
s_wk = casadi.MX.sym('wk', 3);
s_dphik = casadi.MX.sym('dphik', 12);
s_yawk = casadi.MX.sym('yaw', 1);
s_foot_gyrok = casadi.MX.sym('foot_gyro', 12);
s_r = mipo_measurement(s_xk, s_wk, s_phik, s_dphik, s_yawk, s_foot_gyrok, param);
s_R = jacobian(s_r, s_xk);
kf_conf.r = Function('meas',{s_xk, s_wk, s_phik, s_dphik, s_yawk, s_foot_gyrok},{s_r});
kf_conf.dr = Function('meas_jac',{s_xk, s_wk, s_phik, s_dphik, s_yawk, s_foot_gyrok},{s_R});

% foot IMU frame has a transformation wrt foot center 
% FL FR RL RR
param.R_fm_list = {[-1  0  0;
               0  0 -1;
               0 -1  0], 
             [-1   0   0; 
               0   0   1;
               0   1   0],
             [-1  0  0;
               0  0 -1;
               0 -1  0],
             [-1   0  0; 
               0   0  1;
               0   1  0]};
end