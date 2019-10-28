function nlpProb = symProb(Model,MPC,RL,Init)
	nx = Model.nx; nu = Model.nu; N = MPC.N; n = MPC.n;
	syms X [nx,N+1] real;
	syms S [nx,N] 	real;
	syms U [nu,N]	real;
	syms P [n,1]	real;

%Extracting parameters
	Plqr = reshape(P(1:nx*nx),nx,nx)										;%1:4
	V0   = reshape(P(nx*nx+1:nx*nx+1),1,1)									;%5
	xsub = reshape(P(nx*nx+2       		  :nx*nx+1 + nx),nx,1)  			;%6:7
	xtop = reshape(P(nx*nx+2 +   nx		  :nx*nx+1 + 2*nx),nx,1)			;%7:8
	b    = reshape(P(nx*nx+2 + 2*nx		  :nx*nx+1 + 3*nx),nx,1)			;%9:10
	B    = reshape(P(nx*nx+2 + 3*nx 	  :nx*nx+1 + 4*nx),nx,1)     		;%12:13
	F    = reshape(P(nx*nx+2 + 4*nx 	  :nx*nx+1 + 5*nx + nu),nx+nu,1) 	;%14:16
	A 	 = reshape(P(nx*nx+2 + 5*nx + nu  :nx*nx*2+1 + 5*nx + nu),nx,nx) 	;%17:20
	X0 	 = reshape(P(nx*nx*2+2 + 5*nx + nu:nx*nx*2+1 + 6*nx + nu),nx,1) 	;


%Creating constraints and objective
	gamma = RL.gamma; W = Model.W;
	Xlb = MPC.Xlb; Xub = MPC.Xub;	
	g0 = X(:,1) - X0;
	g1 = X(:,2:N+1) - (A*X(:,1:N)+ B*U(:,1:N) + b);
	g2l = [0;1] - xsub + S(:,1:N) + X(:,1:N) ;
	g2t = [1;1] + xtop + S(:,1:N) - X(:,1:N) ;

	obj1 = F'*[X(:,1:N);U(:,1:N)];
	obj2 = 0.5*gamma.^(0:N-1).*(sum(X(:,1:N).^2) + 0.5*U(:,1:N).^2 + W'*S(:,1:N));
	obj3 = V0 + gamma^N/2*X(:,N+1)'*Plqr*X(:,N+1);

	g = [g0;g1(:);g2l(:);g2t(:)];
	obj = sum(obj1) + sum(obj2) + sum(obj3);  

	syms lambda [size(g,1),1] 	real;
	syms chi 	[nu*N,1]		real;

	OPTVariables = [X(:);S(:);U(:)];
	vars = [OPTVariables;lambda;chi;P];
	f = matlabFunction(obj,'Vars',{vars});
%--------------------------------------------------------------------


	L = obj + lambda'*g + chi'*U';
	
    for i = 1:size(OPTVariables,1)
        dLdx(i) = diff(L,OPTVariables(i),1);
    end
    theta = [V0;xsub;xtop;b;B;F;A(:)];
    for i = 1:size(RL.theta,1)
        dLdtheta(i) = diff(L,theta(i),1);
    end
    vars = [OPTVariables;lambda;chi;P];
    dLdx = matlabFunction(dLdx,'Vars',{vars});
    dLdtheta = matlabFunction(dLdtheta,'Vars',{vars});

	nlpProb = struct('f',f,'x',OPTVariables, 'g',g,'p',P,'dLdx',dLdx,'dLdtheta',dLdtheta,'vars',vars);	

end