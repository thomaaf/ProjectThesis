function [nlpProb,RL] = symProb(Model,MPC,RL,Init)
	nx = Model.nx; nu = Model.nu; N = MPC.N; n = MPC.n;
	syms X [nx,N+1] real;
	syms S [nx,N] 	real;
	syms U [nu,N]	real;
	syms P [n,1]	real;

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
	h2 = -1-Xtop + X(:,1:N) - S(:,1:N);
	h3 = -S(:,1:N)

	obj1 = C + gamma^N*T*X(:,N+1)^2;
	obj3 = gamma.^(0:N-1).*(sum(X(:,1:N).^2) + U(:,1:N).^2 + W'*S(:,1:N) + W'*S(:,1:N).^2 + F'*[X(:,1:N);U(:,1:N)]);


	eq = [eq0(:);eq1(:)];
	g  = [g1(:);g2(:)];
	h  = [h1(:);h2(:);h3(:)];

	constraints = [eq;g;h];
	obj = sum(obj1)  + sum(obj3);  

	OPTVariables = [X(:);S(:);U(:)];
	nlpProb = struct('f',obj,'x',OPTVariables, 'g',constraints,'p',P);



	syms chi 	[size(g,1) ,1]		real;
	vars = [OPTVariables;chi;P];
	f = matlabFunction(obj,'Vars',{vars});
%--------------------------------------------------------------------


	L = obj + chi'*g;
	
    for i = 1:size(OPTVariables,1)
        dLdx(i) = diff(L,OPTVariables(i),1);
    end
    %teta = [c;xsub;xtop;f;e;t];
    theta = [C;Xsub;Xtop;F;E;T];
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

%'dLdx',dLdx,'dLdtheta',dLdtheta,'vars',vars,'Dtheta',Dtheta);	
	


end