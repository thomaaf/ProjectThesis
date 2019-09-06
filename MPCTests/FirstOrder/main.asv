%Description
% nx = n states, nu = n inputs, N = Prediciton horizon, T = predict time
% A,B = System matricies; Q,R = Cost matrices, x0 = System initial cond
% tspan = simulationMax time; h = simulation precision; 
% Xref = state references; Xlb,Xub = state lower and upper bounds
% Ulb, Uub = Input lower and upper bounds. 

% N = 2;
% c0: x0 - x_0 = 0
% c1: x1 - Ax0 + Bu0 = 0
% c2: x2 - Ax1 + Bu1 = 0;
% f = x1^2 + x2^2
% X = x0, x1, x2, u0, u1
% 
a = 1; b = 1;
syms x0 x1 x2 u0 u1 l1 l2
f = x1^2 + x2^2;
c1 = x1 - a*x0 - b*u0;
c2 = x2 - a*x1 - b*u1;
L = f - l1*c1 - l2*c2
dLdx = [diff(L,'x0',1),diff(L,'x1',1),diff(L,'x2',1),diff(L,'u0',1),diff(L,'u1',1)]
dLdl = [diff(L,l1,1),diff(L,l2,1)]
%%
nx = 1; nu = 1;
N = 2; T = 0.2; 
A = 1; B = 1; 
Q = diag([1]);
R = 0;
x0 = [1];
tspan = 50;
h = 0.01;
xRef= [-1.5];
Xlb = []; Xub = [];
Ulb = [-5]; Uub = [5];
[x,u,xopt,uopt,t,f] = Simulation(A,B,Q,R,x0,xRef,Xlb,Xub,Ulb,Uub,h,tspan,N,T,nx,nu);
%%
figure(1)
if(0)
    for i = 1:size(x,1)
        subplot(1,2,1)
        %box
            plot([x(i,1)-0.5,x(i,1)+0.5 ],[0 0],'Color','b'); hold on;
            line([x(i,1)-0.5,x(i,1)+0.5 ],[0.5 0.5],'Color','b');
            line([x(i,1)-0.5,x(i,1)-0.5 ],[0 0.5],'Color','b');
            line([x(i,1)+0.5,x(i,1)+0.5 ],[0 0.5],'Color','b'); 
        %Wall 
            plot([-1.5 -1.5],[0 2],'k','LineWidth',12);
        hold off;
        title(t(i))
        axis([-2 2 -1 2])
        title(t(i)); grid on;
        subplot(1,2,2)
        bar(u(i))
        ylim([-10 10])
        pause(0.0001)
    end
end
h = 0.01
figure(1)
clf(1)
axis([0 max(t),-2 2])
grid on
opt = plot(0,0)
inp = plot(0,0)

for i = 1:size(x,1)

   figure(1)
   line([t(i) t(i+1)],[x(i,1) x(i+1,1)]); hold on; grid on
   title(u(i) + " : " + i)
   if ~isnan(xopt(i,:)) 
        delete(opt);
        delete(inp);       
        opt = plot(t(i:T/N/h:i+T/h),xopt(i,:),'b-.');
        inp = plot(t(i:T/N/h:i+(T-T/N)/h),uopt(i,:),'r-.');
        %title(u(i) + " : " + f(i))
       
   end
   pause(0.1)
   axis([0 1,-2 2])

end

