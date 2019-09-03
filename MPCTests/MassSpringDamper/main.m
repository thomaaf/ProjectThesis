%Description
% nx = n states, nu = n inputs, N = Prediciton horizon, T = predict time
% A,B = System matricies; Q,R = Cost matrices, x0 = System initial cond
% tspan = simulationMax time; h = simulation precision; 
% Xref = state references; Xlb,Xub = state lower and upper bounds
% Ulb, Uub = Input lower and upper bounds. 


%%
nx = 2; nu = 1;
N = 4; T = 2; 
A = [0 1; -1 -1];
B = [0;1];    
Q = diag([10 1]);
R = 0;
x0 = [1;0];
tspan = 50;
h = 0.01;
xRef= [-0.5;0];
Xlb = [-1.5;-inf]; Xub = [];
Ulb = [-10]; Uub = [1];
[x,u,xopt,uopt,t] = Simulation(A,B,Q,R,x0,xRef,Xlb,Xub,Ulb,Uub,h,tspan,N,dt,nx,nu);
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

figure(1)
clf(1)
axis([0 size(x,1),-2 2])
grid on
for i = 1:size(x,1)
    
   plot(t(i),x(i,1),'b.'); hold on; grid on
   opt = plot(t(i:),xopt(i,:),'b-.');
   inp = plot(i:i+29,uopt(i,:),'r-.');
   pause(0.1)
   axis([0 size(x,1),-2 2])
   delete(opt);
   delete(inp);
end

