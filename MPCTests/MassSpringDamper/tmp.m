clear
N = 4; nx = 2; nu = 1;
Q = eye(2); R = 1;
A = eye(2); B = [0;1]; syms a1 a2 b

%Number of eqconstraints = N*nx  <=> Number of lagrange multipliers for eq
%% Initialization of variables
syms x   [N+1 nx] real; x=x';        %State variables
syms u   [N nu]   real; u=u';       %Input variables
syms chi [N nx] real;chi = reshape(chi',N*nx,1) %Lagrange multipliers for function eq constraints
z = [x(:,2:end);u];         %Decision variables vector
cost = sym('0'); 
constraints(1:N*nx,1) = sym('0')
%% Calculation of symbolic lagrangian
for k = 1:N % Sum of cost, from 0 to N-1
    cost = cost + z(1:nx,k)'*Q*z(1:nx,k) + z(nx+1:nx+nu,k)'*R*z(nx+1:nx+nu,k);
    constraints((k-1)*nx+1:k*nx) = A*x(:,k) + B*u(:,k) - x(:,k+1);
end
L = cost + chi'*constraints;

%% Calculation of symbolic gradients
J(nx+nu,N,1) = sym('0');
    Jrow(1,N) = sym('0');
for row = 1:nx+nu

    for col = 1:N
        Jrow(1,col) = diff(L,z(row,col),1);
    end
    J(row,:) = Jrow;
end



