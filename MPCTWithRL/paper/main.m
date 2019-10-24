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
    syms e1 e2 real
    syms b1 b2 real
    nx = 2; nu = 1;
    
    A = [1 0.25; 0 1];      Asym = [a1 a2;a3 a4];
    B = [0.0312;0.25];      Bsym = [b1;b2];
    E = [0;0];              Esym = [e1;e2];
    w = [10^2;  10^2];
    Model = struct('nx',nx,'nu',nu,'A',A,'Asym',Asym,'B',B,'Bsym',Bsym,'E',E,'Esym',Esym,'w',w);

%MPC parameters
    N = 10; T = 1; 
	syms V0 real
    syms q1 q2 q3 q4 real
    syms r1          real
    syms f1 f2 f3    real
    syms p1 p2 p3 p4 real
    Vsym = V0;              Vinit = 0;                 
    Q = [1,0;0,1];         Qsym = [q1 q2;q3 q4];
    R = 0.5;                 Rsym = r1;
    f = [0;0;0];            fsym = [f1;f2;f3];
    P = [0 0;0 0];          Psym = [p1 p2;p3 p4];
    nSym = nx*nx*2+1 + 5*nx + nu;
    Xlb = []; Xub = [];
    Ulb = [-1]; Uub = [1];
    MPCParam = struct('N',N,'T',T,'V0',Vinit,'Vsym',Vsym,'Q',Q,'Qsym',Qsym,'R',R,'Rsym',Rsym...
        ,'P',P,'Psym',Psym,'f',f,'fsym',fsym,'Xlb',Xlb,'Xub',Xub,'Ulb'...
        ,Ulb,'Uub',Uub,'nSym',nSym);
    
%RL Parameters
% theta = [Q,A,R,B,E,F]

    gamma = 0.99;
    theta = [q1;q2;q3;q4;a1;a2;a3;a4;r1;b1;b2;e1;e2;f1;f2;f3;Vsym];
    learn = [ 1; 1; 1; 1; 1; 1; 1; 1; 0; 1; 1; 1; 1; 1; 1; 1;1];
    %learn = [ 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    alfa = 0.00001;
    Qrl = [10 0; 0 1]; Rrl = 0;
    RLParam = struct('gamma',gamma,'theta',theta,'alfa',alfa,'learn',learn...
                     ,'Q',Qrl,'R',Rrl);
    
%initial conditions and durations
    
    x0 = [0;0];
    tspan = 20000;
    h = 0.01;
    xRef= [0;0];
    
    InitParam = struct('x0',x0,'tspan',tspan,'h',h,'xRef',xRef);
   
   % MPCParam     = symbolicProblem(Model,MPCParam,RLParam,InitParam,0);
   %[MPCParam.KKT, MPCParam.KKTF ]= symbolicGradiant(MPCParam,MPCParam.vars(1:(N+1)*nx+N*nu,1));
   %[MPCParam.Jtheta,MPCParam.F] = symbolicGradiant(MPCParam,theta);
%%
fprintf("================NEW SIMULATION================\n")
fprintf("================NEW SIMULATION================\n")
fprintf("================NEW SIMULATION================\n")
out = Simulation(Model,MPCParam,RLParam,InitParam);
clearvars -except MPCParam InitParam Model RLParam out
%%
Analyze(out.x,out.u,out.t,out,MPCParam,RLParam,Model)
%%
% evalTime = 20;
% Jnum = zeros(20,1);
% Func = symfun(sum(MPCParam.obj),MPCParam.vars);
% Func = matlabFunction(Func);
% fstring = sprintf('Func(');
% for i = 1:size(out.numsopt,2)
%    fstring = sprintf('%s %s%i%s',fstring,"out.numsopt(t,",i,"),");
% end
% fstring = fstring(1:end-1); fstring = sprintf('%s)',fstring);
% for i = 1:size(Jnum,1)
%     t = (i-1)*20 + 1;
%     Jnum(i) = eval(fstring);
% end
% figure(3)
% clf(3)
% plot(RLParam.gamma*out.V(21:20:end) - out.V(1:20:end-19))
% yyaxis right
% plot(out.TD(1:20:end-1))

%%
%theta = [Q,A,R,B,E,F]
bound = size(out.V,1)-100;
tbound = 101 + bound;
start = 1;
type = '';
figure(2)
clf(2)
subplot(5,2,1)
out.t(1:end-tbound);
plot(  out.theta(start:bound,1:4),type); title("Q"); grid on; legend(string(RLParam.theta(1:4))); hold on;
subplot(5,2,2)
plot(  out.nabla(start:bound,1:4),type); title("Nabla Q"); grid on;legend(string(RLParam.theta(1:4))); hold on;
subplot(5,2,3)
plot(  out.theta(start:bound,5:8),type); title("A"); grid on; legend(string(RLParam.theta(5:8))); hold on;
subplot(5,2,4)
plot(  out.nabla(start:bound,5:8),type); title("Nabla A"); grid on; legend(string(RLParam.theta(5:8))); hold on;
subplot(5,2,5)
plot(  out.theta(start:bound,9:11),type); title("R & B"); grid on; legend(string(RLParam.theta(9:11))); hold on;
subplot(5,2,6)
plot(  out.nabla(start:bound,9:11)),type; title("Nabla R & B"); grid on; legend(string(RLParam.theta(9:11))); hold on;
subplot(5,2,7)
plot(  out.theta(start:bound,12:13),type); title("E"); grid on; legend(string(RLParam.theta(12:13))); hold on;
subplot(5,2,8)
plot(  out.nabla(start:bound,12:13),type); title("Nabla E"); grid on; legend(string(RLParam.theta(12:13))); hold on;
subplot(5,2,9)
plot(  out.theta(start:bound,14:16),type); title("F"); grid on; legend(string(RLParam.theta(14:16))); hold on;
subplot(5,2,10)
plot(  out.nabla(start:bound,14:16),type); title("Nabla F"); grid on; legend(string(RLParam.theta(14:16))); hold on;
%%
figure(5)
clf(5)
moveRMS = dsp.MovingAverage(800);
plot(out.V(start:bound),type); hold on; grid on;
plot(out.reward(start:bound),type);
plot(out.target(start:bound),type);
yyaxis right
plot(out.TD(start:bound),type);
yyaxis left
legend(["V","reward","target",'TD'])
