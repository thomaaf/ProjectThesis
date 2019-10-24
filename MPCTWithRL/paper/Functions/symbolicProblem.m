function MPCParam= symbolicProblem (Model,MPCParam,RLParam,InitParam,type)
%% Calculates a symbolic representation of the optimization problem. Type specifies
%Model
    nx = Model.nx; nu = Model.nu;
    A  = Model.Asym ; B  = Model.Bsym; E = Model.Esym;
%MPC
    N = MPCParam.N; T = MPCParam.T;
    V0 = MPCParam.Vsym;
    Q = MPCParam.Qsym; R = MPCParam.Rsym;
    syms p [1 nx+nx] real;
    Plqr = [p1, p2;p3,p4];
    F = MPCParam.fsym;
%     end
      
%RL
    gamma = RLParam.gamma;
 
%Init
    x0 = InitParam.x0;
    syms x   [nx N+1] real;                 %State variables
    syms u   [nu N] real;                 %Input variables
    syms chi [1 (N+1)*nx] real;                 %Lagrange multipliers for function eq constraints
    %vars = [x;u;chi];                      %Matrix of differentiable variables
    %chi = reshape(chi,(N+1)*nx,1);         %Reshaping the multipliers to correct form
    DCStageCost(1:N,1) = sym('0');          %Vector of Discounted Stagecost 
    LStageCost(1:N,1) = sym('0');          %Vector of Linear, Parametrized Stagecost 
    obj = 0;                                %Objective function -> sum of stageCost    
    g2(1:N*nx + nx,1) = sym('0');       %Vector of all equality constraints

%----------Discretize matricies
    Ak = eye(nx) + T/N*A; Bk = T/N*B; Ek = T/N*E;

%% Calculation of symbolic lagrangian
    g2(1:nx) = x(:,1) - x0;
    for k = 1:N % Sum of cost, from 0 to N-1
        g2(k*nx+1:(k+1)*nx) = x(:,k+1) - (Ak*x(:,k) + Bk*u(:,k) + Ek);
        DCStageCost(k) = 0.5*gamma^(k-1)*( x(:,k)'*Q*x(:,k) + u(:,k)'*R*u(:,k) );
        LStageCost (k) = F'*[x(:,k);u(:,k)];
        obj = obj + DCStageCost(k) + LStageCost(k);
        
        fprintf("\n%i DC: %s \n",k,string(DCStageCost(k)));
        fprintf("%i  L: %s \n",k,string(LStageCost(k)));
        fprintf("%i g2: %s \n",k,string(g2(k)));
        
    end
  
    TMStageCost = V0 + 0.5*gamma^N*x(:,k+1)'*Plqr*x(:,k+1);
    fprintf("\n%i final: %s \n",k,string(TMStageCost ));
    obj = obj + TMStageCost;

    L = obj - chi*g2;
%% Print
    fprintf("Short form\n")
    fprintf("V(s) =\t min z 0.5*\x0194^\x207f\x207a\x00b9*x'P\x2097x +0.5*\x03A3(\x0194^\x207f\x207b\x00b9(x'Qx + u'Ru) + f'[x;u]); \t\tn = 1:N\n")
    fprintf("\t\t st \n\t\t \t x\x2096\x208a\x2081 = x\x2096 + T/N*(Ax\x2096 + Bu\x2096);\t\t\t\t\t\t\t\t\tk = [0:N]\n")
%     fprintf("Discounted Stagecost: \n");disp(DCStageCost);          MPCParam.DCStageCost = DCStageCost; 
%     fprintf("Linear Stagecost: \n");    disp(LPStageCost);          MPCParam.LPStageCost = LPStageCost;
%     fprintf("Terminal Stagecost: \n");  disp(TMStageCost);          MPCParam.TMStageCost = TMStageCost;
%     fprintf("Constraints : \n");        disp(g2);                   MPCParam.constraints = g2;
%     fprintf("Objective function: \n");  disp(children(obj)');       MPCParam.obj = children(obj)';
%     fprintf("Lagrangian; \n");          disp(children(L)');         MPCParam.L = children(L)';
    MPCParam.DCStageCost = DCStageCost; 
    MPCParam.LPStageCost = LStageCost;
    MPCParam.TMStageCost = TMStageCost;
    MPCParam.g2 = g2;
    MPCParam.obj = children(obj)';
    MPCParam.L = children(L)';    
    
    
    optVars = [reshape(x',nx*(N+1),1);reshape(u',nu*(N),1);chi'];

    thetaVars = RLParam.theta;
	MPCParam.vars = [optVars; thetaVars;reshape(MPCParam.Psym',size(MPCParam.Psym,1)*size(MPCParam.Psym,2),1)];
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