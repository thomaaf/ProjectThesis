%Description
% nx = n states, nu = n inputs, N = Prediciton horizon, dt = predict res
% A,B = System matricies; Q,R = Cost matrices, x0 = System initial cond
% tspan = simulationMax time; h = simulation precision; 
% Xref = state references; Xlb,Xub = state lower and upper bounds
% Ulb, Uub = Input lower and upper bounds. 

% nx = 4; nu = 1;
% N = 30; dt = 0.1; 
% A = [0,1,0,0;0,0,4.905,0;0,0,0,1;0,0,14.715,0];
% B = [0;0.5;0;0.5];    
% Q = diag([50,1,1,1]);
% R = 0;
% x0 = [1;0;0.7;0];
% tspan = 5;
% h = 0.01;
% xRef= [0.5;0;0;0];
% Xlb = []; Xub = [];
% Ulb = []; Uub = [];
% [x,u,xopt,uopt,t] = Simulation(A,B,Q,R,x0,xRef,Xlb,Xub,Ulb,Uub,h,tspan,N,dt,nx,nu);

%%
nx = 2; nu = 1;
N = 30; dt = 0.1; 
A = [0 1; -1 -1];
B = [0;1];    
Q = diag([1 1]);
R = 0;
x0 = [1;0];
tspan = 5;
h = 0.01;
xRef= [-0.5;0];
Xlb = []; Xub = [];
Ulb = []; Uub = [];
[x,u,xopt,uopt,t] = Simulation(A,B,Q,R,x0,xRef,Xlb,Xub,Ulb,Uub,h,tspan,N,dt,nx,nu);


Analyze(x,u,t)