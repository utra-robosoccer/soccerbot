function dxdt = M_V_numeric_sys(theta,x,Psi1,Psi2, sigma, sigma_prime, sigma_d_prime)
% M = x(1);
% V = x(2);
sigma_num = ppval(sigma,theta);
sigma_prime_val = ppval(sigma_prime,theta);
sigma_d_prime_val = ppval(sigma_d_prime, theta);

q1 = sigma_num(1);
q2 = sigma_num(2);
sigma_prime1 = sigma_prime_val(1);
sigma_prime2 = sigma_prime_val(2);
sigma_d_prime1 = sigma_d_prime_val(1);
sigma_d_prime2 = sigma_d_prime_val(2);

Psi1_num = Psi1(q1,q2,sigma_prime1,sigma_prime2);
Psi2_num = Psi2(q2,sigma_prime1,sigma_prime2,sigma_d_prime1,sigma_d_prime2);

dxdt = [-2*Psi2_num 0;-Psi1_num 0]*x;


end
