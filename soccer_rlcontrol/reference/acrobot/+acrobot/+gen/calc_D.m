function D = calc_D(i1,i2,l1,lc1,lc2,m1,m2,q2)
%CALC_D
%    D = CALC_D(I1,I2,L1,LC1,LC2,M1,M2,Q2)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    14-Jun-2022 22:36:21

t2 = cos(q2);
t3 = lc2.^2;
t4 = t3.*2.0;
t5 = m2.*t3;
D = reshape([i1+i2+(m2.*(t4+l1.^2.*2.0+l1.*lc2.*t2.*4.0))./2.0+m1.*(l1-lc1).^2,i2+t5+l1.*lc2.*m2.*t2,i2+(m2.*(t4+l1.*lc2.*t2.*2.0))./2.0,i2+t5],[2,2]);
