function dx = Plant(x,u)
x1 = x(1); x2 = x(2); x3 = x(3); x4 = x(4);
m1 = 2; m2 = 1; l = 1; g = 9.81;
A = [m1+m2, -m2*l*cos(x3); -m2*l*cos(x3) m2*l^2];
D = [ 0, m2*l*x4*sin(x3); 0, 0];
G = [0;-g*sin(x3)];
u = [u;0];
dx_prime = A\(-D*[x2;x4] - G + u);
dx_sec   = [x2;x4];

dx = [dx_sec(1);dx_prime(1);dx_sec(2);dx_prime(2)];

end

