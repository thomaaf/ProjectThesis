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

    A = [0 1; -1 -1];
    B = [0;1];    
    Model = struct('nx',nx,'nu',nu,'A',A,'B',B);
%MPC parameters
    N = 5; T = 2; 
    syms q1 q2 q3 q4
    syms r1
    Q = [80,0;0,40];    Qsym = [q1 q2;q3 q4];
    R = 1;              Rsym = r1;
    Xlb = []; Xub = [];
    Ulb = [-4]; Uub = [4];
    MPCParam = struct('N',N,'T',T,'Q',Q,'Qsym',Qsym,'R',R,'Rsym',Rsym,'Xlb',Xlb,'Xub',Xub,'Ulb',Ulb, ...
        'Uub',Uub);
    
%RL Parameters
    gamma = 0.9;
    f = [0;0;0];
    RLParam = struct('gamma',gamma,'f',f);
%initial conditions and durations
    
    x0 = [2;0.5];
    tspan = 6;
    h = 0.01;
    xRef= [0;0];
    
    InitParam = struct('x0',x0,'tspan',tspan,'h',h,'xRef',xRef);
%%
[x,u,xopt,uopt,t] = Simulation(Model,MPCParam,RLParam,InitParam);
%%
Analyze(x,u,t)
