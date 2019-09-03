function dx = PlantMDS(x,u)
x1 = x(1); x2 = x(2); 
A = [0 1;
    -k/m -d/m];
B = [0; 1/m];
dx = A*[x1;x2] + B*u;
end

