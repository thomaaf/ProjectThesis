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
    
    A = [0 1; -1 -1];       Asym = [a1 a2;a3 a4];
    B = [0;1];              Bsym = [b1;b2];
    E = [0;0];              Esym = [e1;e2];
    Model = struct('nx',nx,'nu',nu,'A',A,'Asym',Asym,'B',B,'Bsym',Bsym,'E',E,'Esym',Esym);

%MPC parameters
    N = 5; T = 1; 
    syms q1 q2 q3 q4 real
    syms r1          real
    syms f1 f2 f3    real
    syms p1 p2 p3 p4 real
    Q = [1,0;0,1];          Qsym = [q1 q2;q3 q4];
    R = 0.1;                Rsym = r1;
    f = [0;0;0];            fsym = [f1;f2;f3];
    P = [0 0;0 0];          Psym = [p1 p2;p3 p4];
    nSym = 3*nx^2 + nu^2 + nx*nu + 2*nx + nu
    Xlb = []; Xub = [];
    Ulb = []; Uub = [];
    MPCParam = struct('N',N,'T',T,'Q',Q,'Qsym',Qsym,'R',R,'Rsym',Rsym...
        ,'P',P,'Psym',Psym,'f',f,'fsym',fsym,'Xlb',Xlb,'Xub',Xub,'Ulb'...
        ,Ulb,'Uub',Uub,'nSym',nSym);
    
%RL Parameters
% theta = [Q,A,R,B,E,F]
    syms V0 real
    gamma = 0.9;
    theta = [q1;q2;q3;q4;a1;a2;a3;a4;r1;b1;b2;e1;e2;f1;f2;f3];
    learn = [ 1; 1; 1; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    alfa = 0.01;
    RLParam = struct('gamma',gamma,'theta',theta,'alfa',alfa,'learn',learn);
    
%initial conditions and durations
    
    x0 = [1;0];
    tspan = 600;
    h = 0.01;
    xRef= [0;0];
    
    InitParam = struct('x0',x0,'tspan',tspan,'h',h,'xRef',xRef);
   
    MPCParam     = symbolicProblem(Model,MPCParam,RLParam,InitParam,0);
   [MPCParam.KKT, MPCParam.KKTF ]= symbolicGradiant(MPCParam,MPCParam.vars(1:(N+1)*nx+N*nu,1));
   [MPCParam.Jtheta,MPCParam.F] = symbolicGradiant(MPCParam,theta);
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
% Jnum = zeros(floor(size(out.TD,1)/20),1);
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
bound = 50;
tbound = 101 + bound;
figure(2)
subplot(5,2,1)
plot(out.t(1:end-tbound), out.theta(1:end-bound,1:4)); title("Q"); grid on; legend(string(RLParam.theta(1:4)))
subplot(5,2,2)
plot(out.t(1:end-tbound), out.nabla(1:end-bound,1:4)); title("Nabla Q"); grid on;legend(string(RLParam.theta(1:4)));
subplot(5,2,3)
plot(out.t(1:end-tbound), out.theta(1:end-bound,5:8)); title("A"); grid on; legend(string(RLParam.theta(5:8)));
subplot(5,2,4)
plot(out.t(1:end-tbound), out.nabla(1:end-bound,5:8)); title("Nabla A"); grid on; legend(string(RLParam.theta(5:8)));
subplot(5,2,5)
plot(out.t(1:end-tbound), out.theta(1:end-bound,9:11)); title("R & B"); grid on; legend(string(RLParam.theta(9:11)));
subplot(5,2,6)
plot(out.t(1:end-tbound), out.nabla(1:end-bound,9:11)); title("Nabla R & B"); grid on; legend(string(RLParam.theta(9:11)));
subplot(5,2,7)
plot(out.t(1:end-tbound), out.theta(1:end-bound,12:13)); title("E"); grid on; legend(string(RLParam.theta(12:13)));
subplot(5,2,8)
plot(out.t(1:end-tbound), out.nabla(1:end-bound,12:13)); title("Nabla E"); grid on; legend(string(RLParam.theta(12:13)));
subplot(5,2,9)
plot(out.t(1:end-tbound), out.theta(1:end-bound,14:16)); title("F"); grid on; legend(string(RLParam.theta(14:16)));
subplot(5,2,10)
plot(out.t(1:end-tbound), out.nabla(1:end-bound,14:16)); title("Nabla F"); grid on; legend(string(RLParam.theta(14:16)));
