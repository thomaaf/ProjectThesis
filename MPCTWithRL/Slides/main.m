clear;
addpath("Functions")
%Description

%% Parameters
%Model parameters
%   states,inputs,time,prediction
	
nx = 1; nu = 1;
    
A =    1; 		syms a 		[nx,nx]	real;
B =    0.1;		syms b 		[nx,1]	real;
E =    -0.001; 		syms e 		[nx,1]	real;
Xsub = 0; 		syms xsub 	[nx,1]	real;
Xtop = 1;   	syms xtop 	[nx,1] 	real;
W = 10^2;		

    
Model = struct('A',A,'B',B,'E',E,'Xsub',Xsub,'Xtop',Xtop,'W',W...
			  ,'a',a,'b',b,'e',e,'xsub',xsub,'xtop',xtop,'nx',nx,'nu',nu);


    

%MPC parameters
Q = 1; 			syms q 	[nx,nx] 	real; 
R = 1; 			syms r 	[nu,nu] 	real;
F = [0;0]; 		syms f 	[nx+nu,1] 	real;
C = 0;			syms c  [1,1] 		real;
T = 0;			syms t 				real;

N = 10; 
n  =8; 
MPC = struct('Q',Q,'R',R,'F',F,'C',C,'T',T...
			,'q',q,'r',r,'f',f,'c',c,'t',t...
			,'N',N,'n',n);

%RL Parameters
theta = [c;xsub;xtop;f;e;t];
Theta = [C;Xsub;Xtop;F;E;T];
learn = ones(size(theta,1),1); learn(6) = 1;
gamma = 0.9;
alpha = 1e-6;
RL = struct('theta',theta,'Theta',Theta,'learn',learn,'gamma',gamma,'alpha',alpha);


%initial conditions and durations
X0 = 1; 		syms x0 [nx,1] real;
tspan =10000;

Init = struct('X0',X0,'x0',x0,'tspan',tspan);

nlpProbSym 	= problemDef(Model,MPC,RL,Init,"Sym")
nlpProb 	= problemDef(Model,MPC,RL,Init,"Casadi");


RL.R = rewardFunc(nlpProbSym,Model);
[args,opts] = casadiOptions(Model,MPC,nlpProbSym);

%[nlpProbSym,RL] = symProb(Model,MPC,RL,Init);
fprintf("Initializing simulation...\n")
[RLdata,numdata] = Simulation(Model,MPC,RL,Init,nlpProb,args,opts,nlpProbSym);


clearvars -except MPC Init Model RL out nlpProb nlpProbSym RLdata numdata

%%
figure(1); clf(1)
subplot(3,1,1)
plot(numdata.X1); ylabel("s_1"); grid on;ylim([-0.1 1.1]); hold on;
plot(RLdata.xsub1);
subplot(3,1,2)
plot(numdata.X2); ylabel("s_2"); grid on;ylim([-1.1 1.1]);
subplot(3,1,3)
plot(numdata.U1); ylabel("a"); grid on; ylim([-1.1 1.1]);


figure(2); clf(2)
uicontrol('Style','text','String',"Theta",'Units','normalized','Position',[.28 .95 .42 .05],'FontSize',13,'FontWeight','Bold');  
subplot(3,2,1)
plot(RLdata.e1); hold on; ylabel("e_1"); grid on
subplot(3,2,2)
plot(RLdata.xsub1); hold on;
plot(RLdata.xtop1); ylabel("xsub & xtop"); grid on
subplot(3,2,3)
plot(RLdata.f1); hold on;
plot(RLdata.f2); ylabel("f"); grid on
subplot(3,2,4)
plot(RLdata.c1); ylabel("C"); grid on
subplot(3,2,6)
plot(RLdata.t); hold on; ylabel("T"); grid on


figure(3); clf(3)
uicontrol('Style','text','String',"Nabla",'Units','normalized','Position',[.28 .95 .42 .05],'FontSize',13,'FontWeight','Bold');  

subplot(3,2,1)
plot(RLdata.lam_e1); hold on; ylabel("b"); grid on
subplot(3,2,2)
plot(RLdata.lam_xsub1); hold on;
plot(RLdata.lam_xtop1); ylabel("x_1"); grid on
subplot(3,2,3)
plot(RLdata.lam_f1); hold on;
plot(RLdata.lam_f2); ylabel("f"); grid on
subplot(3,2,4)
plot(RLdata.lam_c1); ylabel("C"); grid on
subplot(3,2,6)
plot(RLdata.lam_t); hold on; ylabel("T"); grid on


movRMS = dsp.MovingAverage(100)
figure(4); clf(4)
subplot(2,1,1); 
plot(RLdata.TD,'-.');ylabel("TD"); grid on; hold on;
plot(movRMS(RLdata.TD))
subplot(2,1,2)
plot(RLdata.V); hold on; plot(RLdata.Target); yyaxis right; plot(RLdata.R); grid on
legend("V","Target")