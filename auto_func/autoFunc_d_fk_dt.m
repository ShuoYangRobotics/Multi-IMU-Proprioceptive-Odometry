function jacobian = autoFunc_d_fk_dt(in1,lc,in3)
%autoFunc_d_fk_dt
%    JACOBIAN = autoFunc_d_fk_dt(IN1,LC,IN3)

%    This function was generated by the Symbolic Math Toolbox version 9.1.
%    24-Feb-2023 13:32:48

d = in3(3,:);
lt = in3(4,:);
t1 = in1(1,:);
t2 = in1(2,:);
t3 = in1(3,:);
t5 = cos(t1);
t6 = cos(t2);
t7 = cos(t3);
t8 = sin(t1);
t9 = sin(t2);
t10 = sin(t3);
t11 = t2+t3;
t12 = cos(t11);
t13 = lt.*t9;
t14 = sin(t11);
t15 = lc.*t12;
t16 = lc.*t14;
t17 = -t15;
t18 = t13+t16;
jacobian = reshape([0.0,-d.*t8+lt.*t5.*t6+lc.*t5.*t6.*t7-lc.*t5.*t9.*t10,d.*t5+lt.*t6.*t8+lc.*t6.*t7.*t8-lc.*t8.*t9.*t10,t17-lt.*t6,-t8.*t18,t5.*t18,t17,-t8.*t16,t5.*t16],[3,3]);
