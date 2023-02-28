function R = quat_to_rot(quat)
    w = quat(1);
    x = quat(2);
    y = quat(3);
    z = quat(4);

    R = [2*(w^2+x^2)-1  2*(x*y-w*z) 2*(x*z+w*y);
         2*(x*y+w*z)  2*(w^2+y^2)-1 2*(y*z-w*x);
         2*(x*z-w*y)  2*(y*z+w*x)   2*(w^2+z^2)-1];
    

end