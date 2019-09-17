%Description
% nx = n states, nu = n inputs, N = Prediciton horizon, T = predict time
% A,B = System matricies; Q,R = Cost matrices, x0 = System initial cond
% tspan = simulationMax time; h = simulation precision; 
% Xref = state references; Xlb,Xub = state lower and upper bounds
% Ulb, Uub = Input lower and upper bounds. 


%%
nx = 2; nu = 1;
N = 10; T = 1; 
A = [0 1; -1 -1];
B = [0;1];    
Q = diag([20 2]);
R = 0;
x0 = [1;0.5];
tspan = 3;
h = 0.01;
xRef= [0;0];
Xlb = []; Xub = [];
Ulb = []; Uub = [];
[x,u,xopt,uopt,t] = Simulation(A,B,Q,R,x0,xRef,Xlb,Xub,Ulb,Uub,h,tspan,N,T,nx,nu);
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
opt = plot(0,0);
inp = plot(0,0);
for i = 1:size(x,1)
   title(u(i))
   plot(t(i),x(i,1),'b.'); hold on; grid on
   if ~isnan(xopt(i,:)) 
        
       delete(opt);
        delete(inp);       
        opt = plot(t(i:T/N/h:i+T/h),xopt(i,:),'b-.');
        inp = plot(t(i:T/N/h:i+(T-T/N)/h),uopt(i,:),'r-.');
       
   end
   pause(0.1)
   axis([0 max(t),-2 2])

end

