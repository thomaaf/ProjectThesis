function nlpProb =  Lagrangian(nlpProb,RL,MPC)
	
	syms P [MPC.n,1]	real;
	syms chi 	[size(nlpProb.g)]		real;
	vars = [nlpProb.x;chi;P];

%--------------------------------------------------------------------
	symFunctions = struct();

	
	Lagrangian = nlpProb.f + chi'*nlpProb.g;
	symFunctions.Lagrangian = children(Lagrangian);

    for i = 1:size(nlpProb.x,1)
        dLdx(i) = diff(Lagrangian,nlpProb.x(i),1);
    end
	symFunctions.dLdx = dLdx;
    for i = 1:size(nlpProb.theta,1)
        dLdtheta(i) = diff(Lagrangian,nlpProb.theta(i),1);
    end
    symFunctions.dLdtheta  = dLdtheta;
    symFunctions.stageCost = nlpProb.stageCost;
    symFunctions.objective = children(nlpProb.f);
    
    nlpProb.dLdx 	 = matlabFunction(dLdx,'Vars',{vars});
    nlpProb.dLdtheta = matlabFunction(dLdtheta,'Vars',{vars});
	nlpProb.f 		 = matlabFunction(nlpProb.f,'Vars',{vars});
	nlpProb.vars 	 = vars;
	nlpProb.stageCost= matlabFunction(sum(nlpProb.stageCost),'Vars',{vars})
	nlpProb.symFunctions = symFunctions;

	%disp("Lagrangian"); 	disp(symFunctions.Lagrangian')
	%disp("dLdx"); 			disp(symFunctions.dLdx')
	%disp("dLdtheta"); 		disp(symFunctions.dLdtheta')
	%disp("StageCost");		disp(symFunctions.stageCost')
	%disp("Objective"); 		disp(symFunctions.objective')
end