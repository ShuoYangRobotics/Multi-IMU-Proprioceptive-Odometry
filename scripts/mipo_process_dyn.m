function xdot = mipo_process_dyn(x,u)

% the process dynamics of attitude-lo-foot filter
% but with additional IMUs on feet
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
%   - foot4 acc bias    (49:57)
%   - time  tk          (52)

% control u 
%   - w      (1:3)      body IMU angular veolocity
%   - a       (4:6)     body IMU acceleration
%   - a1      (7:9)     foot 1 IMU acceleration (already in body frame)
%   - a2      (10:12)   foot 2 IMU acceleration (already in body frame)
%   - a3      (13:15)   foot 3 IMU acceleration (already in body frame)
%   - a4      (16:18)   foot 4 IMU acceleration (already in body frame)
%   - hk       (19)

% dot x 
%   - velocity    
%   - acc    
%   - deuler 
%   - foot1 vel 
%   - foot1 acc 
%   - foot2 vel 
%   - foot2 acc 
%   - foot3 vel 
%   - foot3 acc 
%   - foot4 vel 
%   - foot4 acc 
%   - hk   

if ~iscolumn(x)
    x = x';
end

if ~iscolumn(u)
    u = u';
end

pos = x(1:3);
vel = x(4:6);
euler = x(7:9);

foot1_pos = x(10:12);
foot1_vel = x(13:15);
foot2_pos = x(16:18);
foot2_vel = x(19:21);
foot3_pos = x(22:24);
foot3_vel = x(25:27);
foot4_pos = x(28:30);
foot4_vel = x(31:33);

ba = x(34:36);     % body acc bias
bg = x(37:39);     % gyro bias
% bg = zeros(3,1);     % gyro bias

foot1_ba = x(40:42);     % foot 1 acc bias
foot2_ba = x(43:45);     % foot 2 acc bias
foot3_ba = x(46:48);     % foot 3 acc bias
foot4_ba = x(49:51);     % foot 4 acc bias

w = u(1:3) - bg;     % body angular velocity
a = u(4:6) - ba;     % body linear acceleration


foot1_acc = u(7:9)   - foot1_ba;  % already changed to body frame
foot2_acc = u(10:12) - foot2_ba;  % already changed to body frame
foot3_acc = u(13:15) - foot3_ba;  % already changed to body frame
foot4_acc = u(16:18) - foot4_ba;  % already changed to body frame


deuler = mtx_w_to_euler_dot(euler)*w;

R = euler_to_rot(euler);

acc = R*a -[0;0;9.8];

foot1_acc_w = R*foot1_acc - [0;0;9.8];
foot2_acc_w = R*foot2_acc - [0;0;9.8];
foot3_acc_w = R*foot3_acc - [0;0;9.8];
foot4_acc_w = R*foot4_acc - [0;0;9.8];

xdot = [vel;acc;deuler;
        foot1_vel;foot1_acc_w;
        foot2_vel;foot2_acc_w;
        foot3_vel;foot3_acc_w;
        foot4_vel;foot4_acc_w;
        zeros(3,1);                  % body acc bias
        zeros(3,1);                  % gyro bias
        zeros(3,1);                  % foot 1 acc bias
        zeros(3,1);                  % foot 2 acc bias
        zeros(3,1);                  % foot 3 acc bias
        zeros(3,1);                  % foot 4 acc bias
        1];  
end