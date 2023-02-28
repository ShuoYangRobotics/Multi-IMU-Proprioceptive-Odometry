function dq = quat_from_small_ang(dtheta)
% dtheta must be 3x1
mag = 1/(sqrt(1+dtheta'*dtheta));

dq = mag*[1;dtheta];

end