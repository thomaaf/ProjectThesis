function [nlpProb,args,opts ] = casadiProb(Model,MPC,RL,Init)
	nx = Model.nx; nu = Model.nu; N = MPC.N; n = MPC.n;
	X = casadi.SX.sym('X',nx,N+1);
	S = casadi.SX.sym('S',nx,N);
	U = casadi.SX.sym('U',nu,N);
	P = casadi.SX.sym('P',n,1);

%Extracting parameters
	%teta = [c;xsub;xtop;f;e;t];
	C 	= 	P(1);
	Xsub= 	P(2);
	Xtop= 	P(3);
	F 	= 	P(4:5);
	E 	= 	P(6);
	T 	= 	P(7);
	X0 	= 	P(8);

%Creating constraints and objective
	gamma = RL.gamma; W = Model.W;
	%Xlb = MPC.Xlb; Xub = MPC.Xub;	
	
	eq0 = X(:,1) - X0;
	eq1 = (Model.A*X(:,1:N)+ Model.B*U(:,1:N) + E) - X(:,2:N+1);
	
	g1 =  U - 1;
	g2 = -U - 1;

	h1 =  Xsub - X(:,1:N) - S(:,1:N);
	h2 = -1 - Xtop + X(:,1:N) - S(:,1:N);
	h3 = -S(:,1:N)

	obj1 = C + gamma^N*T*X(:,N+1)^2;
	obj3 = gamma.^(0:N-1).*(sum(X(:,1:N).^2) + U(:,1:N).^2 + W'*S(:,1:N) + W'*S(:,1:N).^2 + F'*[X(:,1:N);U(:,1:N)]);
	

	eq = [eq0(:);eq1(:)];
	g  = [g1(:);g2(:)];
	h  = [h1(:);h2(:);h3(:)];
	
	constraints = [eq;g;h];
	obj = sum(obj1)  + sum(obj3);  

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

