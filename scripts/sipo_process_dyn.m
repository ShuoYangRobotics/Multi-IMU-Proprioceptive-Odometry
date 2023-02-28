function xdot = sipo_process_dyn(x,u)

% the process dynamics of attitude-lo-foot filter
% http://www.roboticsproceedings.org/rss08/p03.pdf
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

% control u 
%   - w      (1:3)      IMU angular veolocity
%   - a       (4:6)     IMU acceleration
%   - hk       (7)


% dot x 
%   - velocity    
%   - acc    
%   - deuler 
%   - d foot
%   - d acc bias
%   - d gyro bias
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

ba = x(22:24);     % acc bias
bg = x(25:27);     % gyro bias
% ba = zeros(3,1);     % acc bias
% bg = zeros(3,1);     % gyro bias


w = u(1:3) - bg;     % body angular velocity
a = u(4:6) - ba;     % body linear acceleration


deuler = mtx_w_to_euler_dot(euler)*w;

R = euler_to_rot(euler);

acc = R*a -[0;0;9.8];

xdot = [vel;
        acc;
        deuler;
        zeros(12,1);
        zeros(3,1);
        zeros(3,1);
        1];  % notice the incrementation of time is done by dyn intergration (dyn_rk4 or dyn_euler)

end