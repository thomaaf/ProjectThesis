%Description
% nx = n states, nu = n inputs, N = Prediciton horizon, T = predict time
% A,B = System matricies; Q,R = Cost matrices, x0 = System initial cond
% tspan = simulationMax time; h = simulation precision; 
% Xref = state references; Xlb,Xub = state lower and upper bounds
% Ulb, Uub = Input lower and upper bounds. 


%% Parameters
%Model parameters
%   states,inputs,time,prediction
	syms a1 a2 a3 a4 real
    nx = 2; nu = 1;
    
    A = [0 1; -1 -1];       Asym = [a1 a2;a3 a4];
    B = [0;1];       
    Model = struct('nx',nx,'nu',nu,'A',A,'Asym',Asym,'B',B);

%MPC parameters
    N = 5; T = 1; 
    syms q1 q2 q3 q4 real
    syms r1          real
    syms f1 f2 f3    real
    syms p1 p2 p3 p4 real
    Q = [0.1,0;0,0.1];      Qsym = [q1 q2;q3 q4];
    R = 0.1;                Rsym = r1;
    f = [0;0;0];            fsym = [f1;f2;f3];
    P = [0 0;0 0];          Psym = [p1 p2;p3 p4];
    Xlb = []; Xub = [];
    Ulb = []; Uub = [];
    MPCParam = struct('N',N,'T',T,'Q',Q,'Qsym',Qsym,'R',R,'Rsym',Rsym,...
        'P',P,'Psym',Psym,'f',f,'fsym',fsym,'Xlb',Xlb,'Xub',Xub,'Ulb',Ulb,'Uub',Uub);
    
%RL Parameters
    syms V0 real
    gamma = 0.9;
    theta = [q1;q2;q3;q4;a1;a2;a3;a4];
    RLParam = struct('gamma',gamma,'theta',theta);
    
%initial conditions and durations
    
    x0 = [1;0];
    tspan = 10000;
    h = 0.01;
    xRef= [0;0];
    
    InitParam = struct('x0',x0,'tspan',tspan,'h',h,'xRef',xRef);
   
    MPCParam     = symbolicProblem(Model,MPCParam,RLParam,InitParam,0);
    MPCParam.KKT = symbolicGradiant(MPCParam,reshape(MPCParam.vars(1:nx+nu,1:N+1)',(nx+nu)*(N+1),1));
    [MPCParam.Jtheta,MPCParam.F] = symbolicGradiant(MPCParam,theta)
%%
fprintf("================NEW SIMULATION================\n")
fprintf("================NEW SIMULATION================\n")
fprintf("================NEW SIMULATION================\n")
out = Simulation(Model,MPCParam,RLParam,InitParam);
clearvars -except MPCParam InitParam Model RLParam out
%%
figure(2)
Analyze(out.x,out.u,out.t,out,MPCParam,RLParam,Model)
