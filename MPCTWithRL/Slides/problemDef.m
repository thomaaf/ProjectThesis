function nlpProb = problemDef(Model,MPC,RL,Init,method)
	nx = Model.nx; nu = Model.nu; N = MPC.N; n = MPC.n;	
	switch method
		case "Casadi"
			fprintf("Assembling casadi problem...\n")
			X = casadi.SX.sym('X',nx,N+1);
			S = casadi.SX.sym('S',nx,N+1);
			U = casadi.SX.sym('U',nu,N);
			P = casadi.SX.sym('P',n,1);
		case "Sym"
			fprintf("Assembling symbolic problem...\n")
			syms X [nx,N+1] real;
			syms S [nx,N+1]	real;
			syms U [nu,N]	real;
			syms P [n,1]	real;
	end

%Extracting parameters
	%teta = [c;xsub;xtop;f;e;t];
	C 	= 	P(1);
	Xsub= 	P(2);
	Xtop= 	P(3);
	F 	= 	P(4:5);
	E 	= 	P(6);
	T 	= 	P(7);
	X0 	= 	P(8);
	theta = [C;Xsub;Xtop;F;E;T];
%Creating constraints and objective
	gamma = RL.gamma; W = Model.W;
	%Xlb = MPC.Xlb; Xub = MPC.Xub;	
	
	eq0 = X(:,1) - X0;
	eq1 = (Model.A*X(:,1:N)+ Model.B*U(:,1:N) + E) - X(:,2:N+1);
	
	g1 =  U - 1;
	g2 = -U - 1;

	h1 =  +  Xsub - X(:,1:N+1) - S(:,1:N+1);
	h2 =  - Xtop + X(:,1:N+1) - S(:,1:N+1);
	h3 = -S(:,1:N+1);

	V_end =  C + T*X(:,N+1)^2 + W*S(:,N+1) + W*S(:,N+1).*S(:,N+1);
	stageCost = (X(:,1:N).*X(:,1:N) + U(:,1:N).*U(:,1:N) + W*S(:,1:N) + W*S(:,1:N).*S(:,1:N) + F'*[X(:,1:N);U(:,1:N)]);

	obj = sum(gamma.^(0:N-1).*stageCost) + gamma^N*V_end;
	eq = [eq0(:);eq1(:)];
	g  = [g1(:);g2(:)];
	h  = [h1(:);h2(:);h3(:)];

	constraints = struct("eq",eq,"g",g,"h",h); 


	OPTVariables = [X(:);S(:);U(:)];
	if method == "Sym"
		nlpProb = struct('f',obj,'x',OPTVariables, 'g',[eq;g;h],'p',P,"constraints",constraints...
						,"theta",theta,'V_star',V_end,'stageCost',stageCost);	
		fprintf("Assembling Lagrangian...\n")
		nlpProb = Lagrangian(nlpProb,RL,MPC);
	else
		nlpProb = struct('f',obj,'x',OPTVariables, 'g',[eq;g;h],'p',P);
	end
end