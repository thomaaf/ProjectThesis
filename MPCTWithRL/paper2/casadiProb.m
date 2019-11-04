function [nlpProb,args,opts ] = casadiProb(Model,MPC,RL,Init)
	nx = Model.nx; nu = Model.nu; N = MPC.N; n = MPC.n;
	X = casadi.SX.sym('X',nx,N+1);
	S = casadi.SX.sym('S',nx,N);
	U = casadi.SX.sym('U',nu,N);
	P = casadi.SX.sym('P',n,1);

%Extracting parameters
	Plqr = reshape(P(1:nx*nx),nx,nx)										;%1:4
	V0   = reshape(P(nx*nx+1:nx*nx+1),1,1)									;%5
	xsub = reshape(P(nx*nx+2       		  :nx*nx+1 + nx),nx,1)  			;%6:7
	xtop = reshape(P(nx*nx+2 +   nx		  :nx*nx+1 + 2*nx),nx,1)			;%7:8
	b    = reshape(P(nx*nx+2 + 2*nx		  :nx*nx+1 + 3*nx),nx,1)			;%9:10
	B    = reshape(P(nx*nx+2 + 3*nx 	  :nx*nx+1 + 4*nx),nx,1)     		;%12:13
	F    = reshape(P(nx*nx+2 + 4*nx 	  :nx*nx+1 + 5*nx + nu),nx+nu,1) 	;%14:16
	A 	 = reshape(P(nx*nx+2 + 5*nx + nu  :nx*nx*2+1 + 5*nx + nu),nx,nx) 	;%17:20
	X0 	 = reshape(P(nx*nx*2+2 + 5*nx + nu:nx*nx*2+1 + 6*nx + nu),nx,1)		; 
	Pshape = [Plqr(:);V0;xsub;xtop;b;B;F;A(:);X0];

%Creating constraints and objective
	gamma = RL.gamma; W = Model.W;
	Xlb = MPC.Xlb; Xub = MPC.Xub;	
	eq0 = X(:,1) - X0;
	eq1 =   (A*X(:,1:N)+ B*U(:,1:N) + b) - X(:,2:N+1);
	
	g1 =  U - 1;
	g2 = -U - 1;

	h1 =  [ 0;-1] + xsub - X(:,1:N) - S(:,1:N);
	h2 = -[ 1; 1] - xtop + X(:,1:N) - S(:,1:N);

	obj1 = V0 + gamma^N/2*X(:,N+1)'*Plqr*X(:,N+1);
	obj2 = F'*[X(:,1:N);U(:,1:N)];
	obj3 = 0.5*gamma.^(0:N-1).*(sum(X(:,1:N).^2) + 0.5*U(:,1:N).^2 + W'*S(:,1:N));

	g = [eq0;eq1(:);g1(:);g2(:);h1(:);h2(:)];
	obj = sum(obj1) + sum(obj2) + sum(obj3);  

	OPTVariables = [X(:);S(:);U(:)];
	nlpProb = struct('f',obj,'x',OPTVariables, 'g',g,'p',P);	
%--------------------------------------------------------------------
	
	lbgeq = zeros(nx*(N+1),1);			ubgeq = zeros(nx*(N+1),1);
	lbgg  = -inf(size([g1(:);g2(:);h1(:);h2(:)],1),1);	ubgg  = zeros(size([g1(:);g2(:);h1(:);h2(:)],1),1);

	args.lbg = [lbgeq;lbgg];
	args.ubg = [ubgeq;ubgg];

	lbX = -inf(nx*(N+1),1); ubX = inf(nx*(N+1),1);
	lbS =zeros(nx*(N),1); ubS = inf(nx*(N),1);
	lbU = -inf(nu*(N),1); 	ubU = inf(nu*(N),1);  
	args.lbx = [lbX;lbS;lbU];
	args.ubx = [ubX;ubS;ubU];


	opts = struct;
	opts.ipopt.max_iter = 100;
	opts.ipopt.print_level = 0;
	opts.print_time = 0;
	opts.ipopt.acceptable_tol = 1e-8;
	opts.ipopt.acceptable_obj_change_tol = 1e-6;
	opts.calc_multipliers = 0; 	

end

