function dJ_drho = autoFunc_dJ_drho(in1,lc,in3)
%autoFunc_dJ_drho
%    dJ_drho = autoFunc_dJ_drho(IN1,LC,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    24-Feb-2023 13:32:49

t1 = in1(1,:);
t2 = in1(2,:);
t3 = in1(3,:);
t5 = cos(t1);
t6 = sin(t1);
t7 = t2+t3;
t8 = cos(t7);
t9 = sin(t7);
t10 = t6.*t9;
t11 = -t8;
t12 = t5.*t9;
t13 = -t10;
dJ_drho = [0.0;t5.*t8;t6.*t8;t11;t13;t12;t11;t13;t12];
