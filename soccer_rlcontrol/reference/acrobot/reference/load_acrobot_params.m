N=2;%DOFs
m = [3.2;0;3.2]; %m1 mH m2
l = [0.2;0.4;0.2;0.4]; %lc1 ll lc2 l2
B = [0;1];
B_p = transpose(null(transpose(B)));

lc1 = l(1);
l1 = l(2);
lc2 = l(3);
l2 = l(4);

g = 9.8;

m1 = m(1);
mH = m(2);
m2 = m(3);
