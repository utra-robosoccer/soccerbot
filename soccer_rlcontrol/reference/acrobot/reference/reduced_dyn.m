function dxdt = reduced_dyn(t,x,Psi1_func,Psi2_func,sigma,sigma_prime, sigma_d_prime)
% function dxdt = acrobot(t,x,l,m,I,phi,B,calc_D,calc_b1,calc_f_func, calc_g_func, calc_e, calc_e_dot, k,Kp,Kd, sld, z_H_max, z_H_min, z_2_max )

dxdt = zeros(2,1);
theta = x(1);
theta_dot = x(2);

sigma_val = ppval(sigma,theta);
sigma_prime_val = ppval(sigma_prime,theta);
sigma_d_prime_val = ppval(sigma_d_prime,theta);


q1 = sigma_val(1);
q2 = sigma_val(2);

Psi1 = Psi1_func(q1,q2,sigma_prime_val(1),sigma_prime_val(2));
Psi2 = Psi2_func(q2,sigma_prime_val(1),sigma_prime_val(2),sigma_d_prime_val(1),sigma_d_prime_val(2));



theta_ddot = Psi1 + Psi2*theta_dot^2;

dxdt = double([theta_dot; theta_ddot]);

end
