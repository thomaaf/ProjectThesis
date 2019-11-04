function [nlpProb,RL] = symProb(Model,MPC,RL,Init)
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
	eq0 = X(:,1) - X0;
	eq1 =   (A*X(:,1:N)+ B*U(:,1:N) + b) - X(:,2:N+1);
	
	g1 =  U - 1;
	g2 = -U - 1;
	g3 = S;
	h1 =  [ 0;-1] + xsub - X(:,1:N) - S(:,1:N);
	h2 = -[ 1; 1] - xtop + X(:,1:N) - S(:,1:N);

	obj1 = V0 + gamma^N/2*X(:,N+1)'*Plqr*X(:,N+1) ;
	obj2 = F'*[X(:,1:N);U(:,1:N)];
	obj3 = 0.5*gamma.^(0:N-1).*(sum(X(:,1:N).^2) + 0.5*U(:,1:N).^2 + W'*S(:,1:N));

	g = [eq0;eq1(:);g1(:);g2(:);h1(:);h2(:);g3(:)];
	obj = sum(obj1) + sum(obj2) + sum(obj3);  

	h = [h1(:,1) + S(:,1) - xsub;h2(:,1) + S(:,1) + xtop];

	syms chi 	[size(g,1) ,1]		real;

	OPTVariables = [X(:);S(:);U(:)];
	vars = [OPTVariables;chi;P];
	f = matlabFunction(obj,'Vars',{vars});
%--------------------------------------------------------------------

	size(g)
	size(chi)
	L = obj + chi'*g;
	
    for i = 1:size(OPTVariables,1)
        dLdx(i) = diff(L,OPTVariables(i),1);
    end
    theta = [V0;xsub;xtop;b;B;F;A(:)];
    for i = 1:size(RL.theta,1)
        dLdtheta(i) = diff(L,theta(i),1);
    end
    Dtheta = dLdtheta;
    dLdx = matlabFunction(dLdx,'Vars',{vars});
    dLdtheta = matlabFunction(dLdtheta,'Vars',{vars});
    h = matlabFunction(h,'Vars',{vars});
	nlpProb = struct('f',f,'x',OPTVariables, 'g',g,'p',P,'dLdx',dLdx,'dLdtheta',dLdtheta,'vars',vars,'Dtheta',Dtheta);	
	RL.h = h;
	RL.W = repmat(Model.W,2,1);
end