function MPCParam= symbolicProblem (Model,MPCParam,RLParam,InitParam,type)
%% Calculates a symbolic representation of the optimization problem. Type specifies
%Model
    nx = Model.nx; nu = Model.nu;
    A  = Model.Asym ; B  = Model.Bsym; E = Model.Esym;
    xsub =Model.xsubsym; xtop = Model.xtopsym;
%MPC
    N = MPCParam.N; T = MPCParam.T;
    V0 = MPCParam.Vsym;
    Q = MPCParam.Qsym; R = MPCParam.Rsym;
    Plqr = MPCParam.Psym;
    F = MPCParam.fsym;

%     end
      
%RL
    gamma = RLParam.gamma;
 
%Init
    X0 = InitParam.x0sym;
    syms X  [nx N+1] real;                 %State variables
    syms S  [nx N]  real
    syms U  [nu N]  real;                 %Input variables
    syms chi [N*nx*3 + 2 1] real;                 %Lagrange multipliers for function eq constraints
    syms lambda [(2*nx+nu)*N+nx 1] real;
    g0 = X(:,1) - X0;
    g1 = X(:,2:N+1) - (A*X(:,1:N)+ B*U(:,1:N) + E);
    g2l = [0;1] - xsub + S(:,1:N) + X(:,1:N) ;
    g2t = [1;1] + xtop + S(:,1:N) - X(:,1:N) ;

    obj1 = F'*[X(:,1:N);U(:,1:N)];
    obj2 = 0.5*gamma.^(0:N-1).*(sum(X(:,1:N).^2) + 0.5*U(:,1:N).^2 + Model.w'*S(:,1:N));
    obj3 = V0 + gamma^N/2*X(:,N+1)'*Plqr*X(:,N+1);


    g = [g0;g1(:);g2l(:);g2t(:)];
    obj = sum(obj1) + sum(obj2) + sum(obj3);

    L = obj + chi'*g + lambda'*[X(:);S(:);U(:)];
%% Print
    
    
    optVars = [X(:);S(:);U(:);chi(:);lambda(:)];
    P = [Plqr(:);RLParam.theta;X0];
	

    MPCParam.vars = [optVars;P];
    MPCParam.optVars = optVars;
    MPCParam.P = P;
    MPCParam.L = L;
    MPCParam.g = g;
    MPCParam.obj = obj;
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