start = 200;
figure(3)
clf(3)
plot(x(:,1),'b')
hold on; grid on; 
plot(start:start+30,xopt(start,:),'b-.')
axis([0 400, -1 1]); yyaxis('right');
plot(u,'k');
plot(start:start+29,uopt(start,:),'k-.')

%%
opti = casadi.Opti();
X = opti.variable(n,N+1);
    CartPos = X(1,:);
    CartVel = X(2,:);
    PendPos = X(3,:);
    PendVel = X(4,:); 
U = opti.variable(1,N);
%Obj and eq Constraints
xnext = casadi.MX(4,N);
for i = 1:N
    xnext(:,i) = X(:,i) + dt*(A*X(:,i) + B*U(i));
    opti.minimize(X(:,i)'*P*X(:,i))    
end
opts = struct; opts.ipopt.print_level= 0; opts.verbose = 0;
opti.solver('ipopt',opts);
% opti.subject_to(); %Clear prev constraints;
% opti.subject_to(X(:,2:end) == xnext)
% opti.subject_to(X(1:n,1) == x(t,:)');

