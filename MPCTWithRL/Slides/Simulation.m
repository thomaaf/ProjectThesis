function [RLdata,numdata] = Simulation(Model,MPC,RL,Init,nlpProb,args,opts,nlpProbSym)
	nx = Model.nx; nu = Model.nu; N = MPC.N;
	RLstore = [string(RL.theta);"lam_"+string(RL.theta);"V";"TD";"R";"Target";"StageCost"];
	numdata = array2table(zeros(Init.tspan-1,size(nlpProbSym.vars,1)),'VariableNames',string(nlpProbSym.vars));
	RLdata  = array2table(zeros(Init.tspan-1,size(RLstore,1)),'VariableNames',RLstore);
	
	solver = casadi.nlpsol('solver', 'ipopt', nlpProb,opts);   
	[plots,movRMS] = plottInit(RLdata,numdata,Init);


	args.X0 = zeros(nx,N+1); args.S0 = zeros(nx,N+1); args.U0 = zeros(nu,N);	
	args.x0 = [args.X0(:);args.S0(:);args.U0(:)]; 
	s = Init.X0;
	theta = RL.Theta;
	t = 1;
	flag = 0;
%Initial solve
	[nums,V,a,args] = solveOpt(Model,MPC,solver,args,theta,Init.X0,nlpProbSym);
	nabla = nlpProbSym.dLdtheta(nums)';


	while t < Init.tspan && flag == 0
		sn = progress(s,a);
 		
 		[numsn,Vn,an,args,flag] = solveOpt(Model,MPC,solver,args,theta,sn,nlpProbSym);
 		nablan = nlpProbSym.dLdtheta(numsn)';
 		[R,target,TD,theta] = RLupdate(RL,s,a,V,Vn,nabla,theta,nums);
 		stageCost = nlpProbSym.stageCost(nums);
 		numdata{t,:} = nums';
 		RLdata{t,:} = [theta;nabla;V;TD;R;target;stageCost]';
		

		s = sn;
		a = an;
		nums = numsn; 
		nabla = nablan;
		V = Vn;
		t = t + 1;
		plotUpdate(plots,RLdata,numdata,movRMS)

	end
	numdata = array2table(numdata{1:t-1,:},'VariableNames',string(nlpProbSym.vars));
	RLdata = array2table(RLdata{1:t-1,:},'VariableNames',RLstore);
end

function sn = progress(s,a)
	sn = s + 0.1*a -0.1*rand;
end

function [nums,V,a,args,flag] = solveOpt(Model,MPC,solver,args,theta,s,nlpProbSym)
	args.p = [theta;s];
	args.p'
	sol = solver('x0',args.x0, 'lbx',args.lbx,'ubx',args.ubx,'lbg',args.lbg,'ubg',args.ubg,'p',args.p);
	flag = 0;
	nums = full([sol.x;sol.lam_g;theta;s]);	
	if abs(max(nlpProbSym.dLdx(nums))) >1e-5
		fprintf("Breach of KKT")
		disp (nlpProbSym.dLdx(nums))
		flag = 1;
	end 	

	args.x0 = full(sol.x);
	a = full(sol.x(find(nlpProbSym.vars == "U1")));
	V = full(sol.f);
end

function [R,target,TD,theta] = RLupdate(RL,s,a,V,Vn,nabla,theta,nums)
	R = RL.R(s,a);
	target = R + RL.gamma*Vn;
	TD =  target - V;
	theta = theta + RL.alpha*TD*nabla.*RL.learn;
end


function [plots,movRMS] = plottInit(RLdata,numdata,Init)
	movRMS = dsp.MovingAverage(100);
	figure(5); clf(5)
	subplot(2,1,1); 
	p1 = scatter(1:Init.tspan -1,RLdata.TD,'.','SizeData',1); 
	hold on; grid on; axis([0 Init.tspan -6 6]); ylabel("TD");
	p2 = plot(movRMS(RLdata.TD));
	

	subplot(2,1,2)
	p3 = scatter(1:Init.tspan -1,RLdata.StageCost,'.','SizeData',1); 
	hold on; grid on; axis([0 Init.tspan 0 10]); ylabel("L");
	p4 = plot(movRMS(RLdata.StageCost));
	
	figure(6); clf(6);
	subplot(3,1,1)
	p5 = plot(RLdata.Target);
 	hold on; grid on;axis([0 Init.tspan -1 10]); ylabel("Value");	 
	p6 = plot(RLdata.V); 
	
	subplot(3,1,2)
	p7 = plot(numdata.X1);  
	grid on; hold on;axis([0 Init.tspan -0.1 1.1]);	ylabel("X_1");
	p8 = plot(RLdata.xsub1,'r');		
	p9 = plot(RLdata.xtop1,'k');		
	

	subplot(3,1,3)
	p10 = plot(numdata.U1);  grid on; hold on;	
	axis([0 Init.tspan -0.1 1.1]);	ylabel("a");	
	plots = [p1;p2;p3;p4;p5;p6;p7;p8;p9;p10];
end
function plotUpdate(plots,RLdata,numdata,movRMS)
	
	plots(1).YData = RLdata.TD;
	plots(2).YData = movRMS(RLdata.TD);
	
	plots(3).YData = RLdata.StageCost;
	plots(4).YData = movRMS(RLdata.StageCost);
	
	plots(5).YData = RLdata.Target;
	plots(6).YData = RLdata.V;
	
	plots(7).YData = numdata.X1;
	plots(8).YData = RLdata.xsub1;
	plots(9).YData = RLdata.xtop1;
	
	plots(10).YData = numdata.U1;
	
	%plots(8).YData = numdata.U1;
	%plots(8).ZData = RLdata.R;
end
