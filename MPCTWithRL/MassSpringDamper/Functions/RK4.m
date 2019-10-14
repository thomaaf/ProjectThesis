function xn = RK4(x,u,dt)
%Numerical Simulation method, Runge kutte of 4th order
    k1 = dt*PlantMDS(x,u);
    k2 = dt*PlantMDS(x + k1/2,u);
    k3 = dt*PlantMDS(x + k2/2,u);
    k4 = dt*PlantMDS(x + k3,u);
    xn = x + 1/6*(k1 + 2*k2 + 2*k3 + k4);
end