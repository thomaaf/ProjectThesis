x0 =[1;0]; u = 0;
t = [0;10]; tc = 0;
l = 1; g = 9.81
[t,x] = ode45(@pendulum,t,x0);

figure(1)
clf(1)
plot(t,l*cos(x(:,1)))
xlabel('x');
ylabel('y');

function dx = pendulum(t,x)
    g = 9.81; l = 1;
    dt = t - evalin('base','tc');
    u = OPT(g,x(1),x(2));
    assignin('base','tc',t);
    %u = LQR(x(1),x(2),g);
    %u =  - 8*x(2) - 5*x(3) - 4*x(1)
    dx(1,1) = x(2);
    dx(2,1) = g*sin(x(1))/l + u;
    u_tmp = evalin('base','u');
    
    assignin('base','u',[u_tmp;u])
    
end
function LQR_U = LQR(x1,x2,g)
    k1 = 2 + g;
    k2 = 3 + g;
    LQR_U = -[k1,k2]*[x1;x2];
end
function OPT_U = OPT(g,x1,x2)
    %Time horizon N;
    %Eulers propagation method
    N = 10;
    A = [0 1; g,0]; B = [0;1]; P = [1 0; 0 1];
    opti = casadi.Opti();
    x = opti.variable(2,N+1); pos = x(1,:); speed = x(2,:);
    u = opti.variable(1,N);
    %t = opti.variable(); %Final time
    dt = 0.1;

    for i = 1:N
        xnext = x(:,i) + dt*(A*x(:,i) + B*u(i));
        opti.subject_to(x(:,i+1)==xnext); %Adding N constraints, where we reduce to error
        opti.minimize(x(:,i)'*P*x(:,i) ); %Summing up the indicidual Costs
    end
    
    opti.subject_to(pos(1) == x1);
    opti.subject_to(speed(1) ==x2);
    opti.set_initial(speed, 1);
    
   
    opti.solver('ipopt');
    sol = opti.solve();
    sol.value(u);
    OPT_U = sol.value(u(1));
 end

% 
% opti = casadi.Opti();
% 
% x = opti.variable();
% y = opti.variable();
% 
% opti.minimize((y-x^2)^2);
% opti.subject_to(x^2+y^2 ==1);
% opti.subject_to(x+y>1);
% 
% opti.solver('ipopt');
% sol = opti.solve()
% x1 