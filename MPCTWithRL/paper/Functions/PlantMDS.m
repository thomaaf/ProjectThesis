function dx = PlantMDS(x,u)
x1 = x(1); x2 = x(2); 

A = [0.9 0.35;
     0   1.1];

B = [0.0813;
     0.2];
 
E = [-1*((rand));
     0 ];
dx = A*[x1;x2] + B*u + E;
end

