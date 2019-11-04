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
    
A = [1 0.25;0 1]; 	syms a 		[nx,nx]	real;
B = [0.0312;0.25];	syms b 		[nx,1]	real;
E = [0;0]; 			syms e 		[nx,1]	real;
Xsub = [0;0]; 		syms xsub 	[nx,1]	real;
Xtop = [0;0];   	syms xtop 	[nx,1] 	real;
W = [10^2;  10^2]*1;		

    
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
learn = ones(size(theta,1),1); learn(1) = 1;
gamma = 0.9;
alpha = 1e-6;
RL = struct('theta',theta,'Theta',Theta,'learn',learn,'gamma',gamma,'alpha',alpha);


%initial conditions and durations
X0 = [0;0]; 		syms x0 [nx,1] real;
tspan = 100000;

Init = struct('X0',X0,'x0',x0,'tspan',tspan);

[nlpProb,args,opts ] = casadiProb(Model,MPC,RL,Init);
[nlpProbSym,RL] = symProb(Model,MPC,RL,Init);
[RLdata,numdata] = Simulation(Model,MPC,RL,Init,nlpProb,args,opts,nlpProbSym);


clearvars -except MPC Init Model RL out nlpProb nlpProbSym RLdata numdata

%%
figure(1); clf(1)
subplot(3,1,1)
plot(numdata.X1_1); ylabel("s_1"); grid on;ylim([-0.1 1.1]); hold on;
plot(0 + RLdata.xsub1);
plot(numdata.S1_1);
subplot(3,1,2)
plot(numdata.X2_1); ylabel("s_2"); grid on;ylim([-1.1 1.1]);
subplot(3,1,3)
plot(numdata.U1); ylabel("a"); grid on; ylim([-1.1 1.1]);


figure(2); clf(2)
subplot(3,2,1)
plot(RLdata.e1); hold on;
plot(RLdata.e2); ylabel("b"); grid on

subplot(3,2,2)
plot(RLdata.xsub1); hold on;
plot(RLdata.xtop1); ylabel("x_1"); grid on

subplot(3,2,3)
plot(RLdata.f1); hold on;
plot(RLdata.f2);
plot(RLdata.f3); ylabel("f"); grid on

subplot(3,2,4)
plot(RLdata.v01); ylabel("Q0"); grid on

subplot(3,2,5)
plot(RLdata{:,13:16}); ylabel("A"); grid on

subplot(3,2,6)
plot(RLdata.b1); hold on;
plot(RLdata.b2); ylabel("B"); grid on



figure(3); clf(3)
subplot(3,2,1)
plot(RLdata.lam_e1); hold on;
plot(RLdata.lam_e2); ylabel("b"); grid on

subplot(3,2,2)
plot(RLdata.lam_xsub1); hold on;
plot(RLdata.lam_xtop1); ylabel("x_1"); grid on

subplot(3,2,3)
plot(RLdata.lam_f1); hold on;
plot(RLdata.lam_f2);
plot(RLdata.lam_f3); ylabel("f"); grid on

subplot(3,2,4)
plot(RLdata.lam_v01); ylabel("Q0"); grid on

subplot(3,2,5)
plot(RLdata{:,29:32}); ylabel("A"); grid on

subplot(3,2,6)
plot(RLdata.lam_b1); hold on;
plot(RLdata.lam_b2); ylabel("B"); grid on

movRMS = dsp.MovingAverage(100)
figure(4); clf(4)
subplot(2,1,1); 
plot(RLdata.TD,'-.');ylabel("TD"); grid on; hold on;
plot(movRMS(RLdata.TD))
subplot(2,1,2)
plot(RLdata.V); hold on; plot(RLdata.Target); yyaxis right; plot(RLdata.R); grid on
legend("V","Target")