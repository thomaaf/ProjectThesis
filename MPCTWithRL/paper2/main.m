clear;
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
    
A = [0.9 0.35;0 1.1]; 	syms a 		[nx,nx]	real;
B = [0.0813;0.2];	syms b 		[nx,1]	real;
E = [-.1;0]; 			syms e 		[nx,1]	real;
Xsub = [0;0]; 		syms xsub 	[nx,1]	real;
Xtop = [0;0];   	syms xtop 	[nx,1] 	real;
W = [10^2;  10^2];		

    
Model = struct('A',A,'B',B,'E',E,'Xsub',Xsub,'Xtop',Xtop,'W',W...
			  ,'a',a,'b',b,'e',e,'xsub',xsub,'xtop',xtop,'nx',nx,'nu',nu);


    

%MPC parameters
Q = [0.5 0; 0 0.5]; syms q 	[nx,nx] 	real; 
R = 0.25; 			syms r 	[nu,nu] 	real;
F = [0;0;0]; 		syms f 	[nx+nu,1] 	real;
P = [0 0; 0 0]; 	syms p 	[nx,nx] 	real;
V0 = 0;				syms v0 [1,1] 		real;

N = 10; 
Xlb = [-inf;-inf;-1]; Xub = [inf;inf;1];
n = nx^2 + 16 + nx;

MPC = struct('Q',Q,'R',R,'F',F,'P',P,'V0',V0...
			,'q',q,'r',r,'f',f,'p',P,'v0',v0...
			,'N',N,'Xlb',Xlb,'Xub',Xub,'n',n);

%RL Parameters
theta = [v0;xsub;xtop;e;b;f;a(:)];
Theta = [V0;Xsub;Xtop;E;B;F;A(:)];
learn = ones(size(theta,1),1);
gamma = 0.99;
alpha = 0.0001;
RL = struct('theta',theta,'Theta',Theta,'learn',learn,'gamma',gamma,'alpha',alpha);


%initial conditions and durations
X0 = [0;0]; 		syms x0 [nx,1] real;
tspan = 100;

Init = struct('X0',X0,'x0',x0,'tspan',tspan);

[nlpProb,args,opts ] = casadiProb(Model,MPC,RL,Init);
nlpProbSym = symProb(Model,MPC,RL,Init);
[RLdata,numdata] = Simulation(Model,MPC,RL,Init,nlpProb,args,opts,nlpProbSym);
   
  