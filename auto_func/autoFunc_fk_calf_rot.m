function fk_calf_rot = autoFunc_fk_calf_rot(in1,lc,in3)
%autoFunc_fk_calf_rot
%    FK_CALF_ROT = autoFunc_fk_calf_rot(IN1,LC,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    24-Feb-2023 13:32:48

t1 = in1(1,:);
t2 = in1(2,:);
t3 = in1(3,:);
t5 = cos(t1);
t6 = sin(t1);
t7 = t2+t3;
t8 = cos(t7);
t9 = sin(t7);
fk_calf_rot = reshape([t8,t6.*t9,-t5.*t9,0.0,t5,t6,t9,-t6.*t8,t5.*t8],[3,3]);
