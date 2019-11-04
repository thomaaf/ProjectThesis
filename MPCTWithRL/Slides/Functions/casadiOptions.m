function [args,opts] = casadiOptions(Model,MPC,nlpProb)
	nx = Model.nx; nu = Model.nu; N = MPC.N;
	lbsys = zeros(size(nlpProb.constraints.eq)); ubsys = zeros(size(nlpProb.constraints.eq));

	lbg = -inf(size([nlpProb.constraints.g;nlpProb.constraints.h]));
	ubg =  zeros(size([nlpProb.constraints.g;nlpProb.constraints.h]));

	args.lbg = [lbsys;lbg];
	args.ubg = [ubsys;ubg];

	lbX = -inf(nx*(N+1),1); ubX = inf(nx*(N+1),1);
	lbS =zeros(nx*(N),1); ubS = inf(nx*(N),1);
	lbU = -inf(nu*(N),1); 	ubU = inf(nu*(N),1);  
	args.lbx = -inf(size(nlpProb.x));
	args.ubx = inf(size(nlpProb.x));



	opts = struct;
	opts.ipopt.max_iter = 100;
	opts.ipopt.print_level = 0;
	opts.print_time = 0;
	opts.ipopt.acceptable_tol = 1e-8;
	opts.ipopt.acceptable_obj_change_tol = 1e-6;
	opts.calc_multipliers = 0; 	
