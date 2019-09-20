%Description
% nx = n states, nu = n inputs, N = Prediciton horizon, T = predict time
% A,B = System matricies; Q,R = Cost matrices, x0 = System initial cond
% tspan = simulationMax time; h = simulation precision; 
% Xref = state references; Xlb,Xub = state lower and upper bounds
% Ulb, Uub = Input lower and upper bounds. 


%%

syms q1 q2 q3 q4
syms r1
nx = 2; nu = 1;
N = 10; T = 2; 
A = [0 1; -1 -1];
B = [0;1];    
Q = [8000000,0;0,40];
Qsym = [q1 q2;q3 q4];
R = 0;
Rsym = r1;
x0 = [1;0.5];
tspan = 6;
h = 0.01;
xRef= [0;0];
Xlb = []; Xub = [];
Ulb = [-4]; Uub = [4];
[x,u,xopt,uopt,t] = Simulation(A,B,Q,Qsym,R,Rsym,x0,xRef,Xlb,Xub,Ulb,Uub,h,tspan,N,T,nx,nu);
%%

%plot(t,xopt(size(xopt,1)/2+1:10:end,'b-.');
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
%colors = ['k.','k-.','y.','y-.'];
%colors = ['b.','b-.','r.','r-.'];
colors = ['m.','m-.','c.','c-.'];

h = 0.01;
figure(2)
subplot(2,1,1)
axis([0 max(t),-2 2])
grid on
opt = plot(0,0);
inp = plot(0,0);
for i = 1:size(x,1)
   title(u(i))
   subplot(2,1,1)
   p1 = plot(t(1:i),x(1:i,1),colors(1:2)); hold on; grid on
   axis([0 max(t),-2 2])
   subplot(2,1,2)
   
   p2 = plot(t(1:i),u(1:i,1),colors(6:7)); 
   axis([0 max(t),-2 2]); hold on; grid on
   if ~isnan(xopt(i,:)) 
        delete(opt);
        delete(inp); 
        subplot(2,1,1)
        opt = plot(t(i:T/N/h:i+T/h),xopt(i,:),colors(3:5));
        subplot(2,1,2)
        inp = plot(t(i:T/N/h:i+(T-T/N)/h),uopt(i,:),colors(8:10));
       
   end
   %axis([0 max(t),-2 2])
   pause(0.01)
   delete(p1)
   delete(p2)

end
   title(u(i))
   subplot(2,1,1)
   p1 = plot(t(1:i),x(1:i,1),colors(1:2)); hold on; grid on
   subplot(2,1,2)
   p2 = plot(t(1:i),u(1:i,1),colors(6:7)); 
