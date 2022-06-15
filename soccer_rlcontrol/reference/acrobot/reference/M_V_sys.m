function dxdt = M_V_sys(theta,x,Psi1,Psi2)
% M = x(1);
% V = x(2);

dxdt = [-2*Psi2(theta) 0;-Psi1(theta) 0]*x;


end
