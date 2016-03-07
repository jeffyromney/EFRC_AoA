function Q = calcQ(daxNoise,dayNoise,dazNoise,dvxNoise,dvyNoise,dvzNoise,q0,q1,q2,q3)
%CALCQ
%    Q = CALCQ(DAXNOISE,DAYNOISE,DAZNOISE,DVXNOISE,DVYNOISE,DVZNOISE,Q0,Q1,Q2,Q3)

%    This function was generated by the Symbolic Math Toolbox version 5.8.
%    26-Dec-2014 15:40:28

t3 = q0.^2;
t4 = q1.^2;
t5 = q2.^2;
t6 = q3.^2;
t2 = t3+t4+t5+t6;
t7 = t2.^2;
t11 = q0.*q3.*2.0;
t12 = q1.*q2.*2.0;
t8 = t11-t12;
t13 = q0.*q2.*2.0;
t14 = q1.*q3.*2.0;
t9 = t13+t14;
t10 = t3+t4-t5-t6;
t15 = q0.*q1.*2.0;
t16 = t11+t12;
t17 = dvxNoise.*t10.*t16;
t18 = t3-t4+t5-t6;
t19 = q2.*q3.*2.0;
t20 = t15-t19;
t21 = t15+t19;
t22 = t3-t4-t5+t6;
t23 = t13-t14;
t24 = dvzNoise.*t9.*t22;
t25 = t24-dvxNoise.*t10.*t23-dvyNoise.*t8.*t21;
t26 = dvyNoise.*t18.*t21;
t27 = t26-dvxNoise.*t16.*t23-dvzNoise.*t20.*t22;
Q = reshape([daxNoise.*t7,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,dayNoise.*t7,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,dazNoise.*t7,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,dvxNoise.*t10.^2+dvyNoise.*t8.^2+dvzNoise.*t9.^2,t17-dvyNoise.*t8.*t18-dvzNoise.*t9.*t20,t25,0.0,0.0,0.0,0.0,0.0,0.0,t17-dvzNoise.*t9.*(t15-q2.*q3.*2.0)-dvyNoise.*t8.*t18,dvxNoise.*t16.^2+dvyNoise.*t18.^2+dvzNoise.*t20.^2,t27,0.0,0.0,0.0,0.0,0.0,0.0,t25,t27,dvxNoise.*t23.^2+dvyNoise.*t21.^2+dvzNoise.*t22.^2,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[9, 9]);
