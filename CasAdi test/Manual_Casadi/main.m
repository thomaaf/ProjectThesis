%Description
%     n:  nr of states;
%     N:  Prediction horizon;
%     dt: Prediciton time precision;
%     A:  State matrix;
%     B:  Input matrix;
%     P:  Cost function;
%     h:  Simulation delta t
%     

n = 4;
N = 30; dt = 0.1; 
A = [0,1,0,0;0,0,4.905,0;0,0,0,1;0,0,14.715,0];
B = [0;0.5;0;0.5];    
P = diag([50,1,1,1]);
x0 = [1;0;0.7;0];
tspan = 4; h = 0.01;
%[x,u,t,uopt,xopt] = 
Simulation(A,B,P,N,dt,n,tspan,h,x0);