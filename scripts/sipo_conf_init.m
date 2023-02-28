function [kf_conf,param] = sipo_conf_init(param)
import casadi.*

% still use casadi for calculting jacobians

kf_conf = {};

kf_conf.state_size = 28;   
% state x 
%   - position    (1:3)
%   - velocity    (4:6)
%   - euler angle (7:9)
%   - foot1       (10:12)
%   - foot2       (13:15)
%   - foot3       (16:18)
%   - foot4       (19:21)
%   - acc bias    (22:24)
%   - gyro bias   (25:27)
%   - time  tk    (28)
  
param.sipo_meas_per_leg = 7;  % 3 3 1 
kf_conf.meas_size = param.sipo_meas_per_leg * param.num_leg + 1; % yaw
param.sipo_meas_size = kf_conf.meas_size;

kf_conf.ctrl_size = 7;  
% control u 
%   - w      (1:3)      IMU angular veolocity
%   - a       (4:6)     IMU acceleration
%   - hk       (7)


% process noise
kf_conf.Q1 = diag([1e-4*ones(3,1); 
           1e-2*ones(3,1); 
           1e-6*ones(3,1);
           1e-3*ones(12,1);  % foot pos
           1e-4*ones(3,1);    % acc bias random walk
           1e-4*ones(3,1);    % gyro bias random walk
           0;]);                  % do not use tk in KF

% control noise 
kf_conf.Q2 = diag([1e-4*ones(3,1);   % acc noise
                   1e-4*ones(3,1);   % gyro noise
                   0]);

kf_conf.R = 1e-2*eye(kf_conf.meas_size);

% get process jacobians
s_xk = casadi.MX.sym('X_k', kf_conf.state_size);
s_uk = casadi.MX.sym('Uk', kf_conf.ctrl_size);
s_uk1 = casadi.MX.sym('Uk1', kf_conf.ctrl_size);
s_dt = casadi.MX.sym('dt', 1);
s_f = dyn_rk4(s_xk , s_uk, s_uk1, s_dt, @sipo_process_dyn); % EKF process
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
s_r = sipo_measurement(s_xk, s_wk, s_phik, s_dphik, s_yawk, param);
s_R = jacobian(s_r, s_xk);
kf_conf.r = Function('meas',{s_xk, s_wk, s_phik, s_dphik, s_yawk},{s_r});
kf_conf.dr = Function('meas_jac',{s_xk, s_wk, s_phik, s_dphik, s_yawk},{s_R});


end