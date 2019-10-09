function MPCParam= symbolicProblem (Model,MPCParam,RLParam,InitParam,type)
%% Calculates a symbolic representation of the optimization problem. Type specifies
%% if the LQR matrix should be symbolic (0) or not(1)
%Model
    nx = Model.nx; nu = Model.nu;
    A  = Model.Asym ; B  = Model.B; 
%MPC
    N = MPCParam.N; T = MPCParam.T;
%     if type == 1
%         Q = MPCParam.Q; R = MPCParam.R;
%         [K,Plqr,e] = lqr(A,B,Q,R,[]);
%         f = MPCParam.f; 
%     else
    Q = MPCParam.Qsym; R = MPCParam.Rsym;
    syms p [1 nx+nx] real;
    Plqr = [p1, p2;p3,p4];
    f = MPCParam.fsym;
%     end
      
%RL
    gamma = RLParam.gamma;
 
%Init
    x0 = InitParam.x0;
    syms x   [N+1 nx] real;   x=x';         %State variables
    syms u   [N+1 nu] real; u=u';           %Input variables
    syms chi [N+1 nx] real;                 %Lagrange multipliers for function eq constraints
    z = [x(:,2:end);u(1:end-1)];            %Decision variables vector(That are in the cost)
    vars = [x;u;chi'];                      %Matrix of differentiable variables
    chi = reshape(chi',(N+1)*nx,1);         %Reshaping the multipliers to correct form
    DCStageCost(1:N,1) = sym('0');          %Vector of Discounted Stagecost 
    LPStageCost(1:N,1) = sym('0');          %Vector of Linear, Parametrized Stagecost 
    obj = 0;                                %Objective function -> sum of stageCost    
    constraints(1:N*nx,1) = sym('0');       %Vector of all equality constraints

    
    
%% Calculation of symbolic lagrangian

    for k = 1:N % Sum of cost, from 0 to N-1
        constraints((k-1)*nx+1:k*nx) = x(:,k) +  T/N*(A*x(:,k) + B*u(:,k)) - x(:,k+1);
        DCStageCost(k) = 0.5 * gamma^(k-1) * (x(:,k)'*Q*x(:,k) + u(:,k)'*R*u(:,k));
        LPStageCost(k) = f'*[x(:,k);u(:,k)];
        obj = obj + DCStageCost(k) + LPStageCost(k);
        
        
    end
    TMStageCost = 0.5*gamma^(k+1)*(x(:,k+1)'*Plqr*x(:,k+1));
    obj = obj + TMStageCost;
    constraints = [x0 - x(:,1);constraints];
    L = obj - chi'*constraints;
%% Print
    fprintf("Short form\n")
    fprintf("V(s) =\t min z 0.5*\x0194^\x207f\x207a\x00b9*x'P\x2097x +0.5*\x03A3(\x0194^\x207f\x207b\x00b9(x'Qx + u'Ru) + f'[x;u]); \t\tn = 1:N\n")
    fprintf("\t\t st \n\t\t \t x\x2096\x208a\x2081 = x\x2096 + T/N*(Ax\x2096 + Bu\x2096);\t\t\t\t\t\t\t\t\tk = [0:N]\n")
    fprintf("Long form\n")
    fprintf("Discounted Stagecost: \n");disp(DCStageCost);          MPCParam.DCStageCost = DCStageCost; 
    fprintf("Linear Stagecost: \n");    disp(LPStageCost);          MPCParam.LPStageCost = LPStageCost;
    fprintf("Terminal Stagecost: \n");      disp(TMStageCost);           MPCParam.TMStageCost = TMStageCost;
    fprintf("Constraints : \n");        disp(constraints);          MPCParam.constraints = constraints;
    fprintf("Objective function: \n");  disp(children(obj)');       MPCParam.obj = children(obj)';
    fprintf("Lagrangian; \n");          disp(children(L)');         MPCParam.L = children(L)';
    MPCParam.vars = vars;
end




% 
% function  LagrangianX(Model,MPCParam,RLParam,InitParam,L,xx,uu,sol,vars,obj)
%     N = MPCParam.N;
%     nx = Model.nx; nu = Model.nu;
%     nums = full([xx;[uu,0];reshape(sol.lam_g,2,N+1)]);% Matrix of numerical values for diff variables
% %% Calculation of symbolic gradients
%     J(nx+nu,N+1,1) = sym('0');
%         Jrow(1,N+1) = sym('0');
%     for row = 1:nx+nu
%         for col = 1:N+1
%             Jrow(1,col) = diff(L,vars(row,col),1);
%         end
%         J(row,:) = Jrow;
%     end
%     fprintf("Solution cost                 : %6.2f \n",full(sol.f))
%     fprintf("Solution cost - evaluated cost: %6.2f \n",full(sol.f)-eval(subs(obj,vars,nums)))
%     fprintf("Symbolic Lagrangian           : %s \n",L)
%     fprintf("Symbolic Gradient at optimum  : \n \n")
%         disp(J)
%      fprintf("Numerical Gradient at optimum : \n \t")
%          disp(eval(subs(J,vars,nums)))
% 
% end
% 
% function nabla = LagrangianPhi(Model,MPCParam,RLParam,InitParam,L,xx,uu,sol,vars)
%     N = MPCParam.N;
%     nx = Model.nx; nu = Model.nu;
%     Qsym = MPCParam.Qsym; Rsym = MPCParam.Rsym;
%     nums = full([xx;[uu,0];reshape(sol.lam_g,2,N+1)]);% Matrix of numerical values for diff variables
%     %phi = [reshape(Qsym,nx*nx,1);reshape(Rsym,nu*nu,1) ]; 
%     phi = [Qsym(1); Qsym(4); Rsym]
% %% Calculation of symbolic gradients
%     J(size(phi,1),1) = sym('0');
%     for i = 1:size(phi,1)
%         J(i) = diff(L,phi(i),1);
%     end    
%     fprintf("Symbolic Lagrangian           : %s \n",L)
%     fprintf("Symbolic GradientPhi          : \n \n")
%         disp(J)
%     fprintf("Numerical GradientPhi         : \n ")
%         disp(eval(subs(J,vars,nums)))
%     nabla = eval(subs(J,vars,nums));
%     
%     
%     
% end
%    