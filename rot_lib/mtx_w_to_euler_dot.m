function mtx = mtx_w_to_euler_dot(euler)
    % convert euler angle theta to the euler kinematics formula
    
r = euler(1);
p = euler(2);
y = euler(3);

mtx = [1, (sin(p)*sin(r))/cos(p), (cos(r)*sin(p))/cos(p);
 0,                 cos(r),                -sin(r);
 0,          sin(r)/cos(p),          cos(r)/cos(p)];

end