function R = euler_to_rot(euler)
    % convert r p y euler angle to rotation matrix

r = euler(1);
p = euler(2);
y = euler(3);

Ry = [cos(y) -sin(y) 0;
      sin(y) cos(y)  0;
      0    0  1];
Rp = [cos(p)  0  sin(p);
      0   1   0;
      -sin(p)  0 cos(p)];
  
Rr = [1  0  0 ;
      0  cos(r)  -sin(r);
      0  sin(r)   cos(r)];
R = Ry*Rp*Rr;

end