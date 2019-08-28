syms m1 m2 l g x1 x2 x3 x4 F
A = [m1 + m2, -m2*l*cos(x3); -m2*l*cos(x3) m2*l^2];
D = [m2*l*x4^2*sin(x3);0];
G = [0;-g*sin(x3)];
sys_prime = A\(-D -G + F);
sys_sec   = [x2;x4];
sys = [sys_sec(1);sys_prime(1);sys_sec(2); sys_prime(2)];
J = [diff(sys,x1,1),diff(sys,x2,1),diff(sys,x3,1),diff(sys,x4,1)];
B = diff(sys,F,1);
x1 = 0; x2 = 0; x3 = 0; x4 = 0; g = 9.81; m1 = 2; m2 = 1; l = 1;
A = eval(subs(J))
B = eval(subs(B))
