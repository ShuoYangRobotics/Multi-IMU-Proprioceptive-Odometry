function mtx = mtx_w_to_quat_dot(w)
    % convert angular velocity to 4x4 matrix 
    % make sure w is 3x1
mtx = [0 -w';
       w -skew(w)];
end