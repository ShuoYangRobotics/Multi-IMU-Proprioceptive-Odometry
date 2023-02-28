function fk_hip_pos = autoFunc_fk_hip_pos(in1,lc,in3)
%autoFunc_fk_hip_pos
%    FK_HIP_POS = autoFunc_fk_hip_pos(IN1,LC,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    24-Feb-2023 13:32:48

d = in3(3,:);
ox = in3(1,:);
oy = in3(2,:);
t1 = in1(1,:);
fk_hip_pos = [ox;oy+(d.*cos(t1))./2.0;(d.*sin(t1))./2.0];
