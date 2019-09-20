%Description
% nx = n states, nu = n inputs, N = Prediciton horizon, T = predict time
% A,B = System matricies; Q,R = Cost matrices, x0 = System initial cond
% tspan = simulationMax time; h = simulation precision; 
% Xref = state references; Xlb,Xub = state lower and upper bounds
% Ulb, Uub = Input lower and upper bounds. 


%% Parameters
%Model parameters
%   states,inputs,time,prediction
    nx = 2; nu = 1;
    N = 10; T = 2; 
    A = [0 1; -1 -1];
    B = [0;1];    

%MPC parameters
    syms q1 q2 q3 q4
    syms r1
    Q = [80,0;0,40];    Qsym = [q1 q2;q3 q4];
    R = 0;              Rsym = r1;
    Xlb = []; Xub = [];
    Ulb = [-4]; Uub = [4];
%initial conditions and durations
    x0 = [1;0.5];
    tspan = 6;
    h = 0.01;
    xRef= [0;0];
%%
[x,u,xopt,uopt,t] = Simulation(A,B,Q,Qsym,R,Rsym,x0,xRef,Xlb,Xub,Ulb,Uub,h,tspan,N,T,nx,nu);
%%

