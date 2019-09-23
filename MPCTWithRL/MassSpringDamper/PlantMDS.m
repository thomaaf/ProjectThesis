function dx = PlantMDS(x,u)
x1 = x(1); x2 = x(2); 
k = 1; m = 1; d = 1;
A = [0 1;
    -k/m -d/m];
B = [0; 1/m];
E = [0;(randn)*0];
dx = A*[x1;x2] + B*u + E;
end

