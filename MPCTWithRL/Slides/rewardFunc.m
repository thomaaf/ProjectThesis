function R = rewardFunc(nlpProb,Model)
	xsub = 0;
	xtop = 1;
	a = nlpProb.x(find(nlpProb.x == "U1"));
	x = nlpProb.x(find(nlpProb.x == "X1"));
	h1 =  xsub - x;
	h2 = -xtop + x;
	R = 1*(nlpProb.x(1)-0)'*(nlpProb.x(1)-0) + 0.25*a'*a + [Model.W;Model.W]'*max(0,[h1;h2]);
	R = matlabFunction(R,'Vars',{[x],[a]});
	
	x = -1:0.01:2; u = -1:0.01:1; Rs = zeros(size(x,2),size(u,2));
	for xi = 1:size(x,2)
		for ui = 1:size(u,2)
			Rs(xi,ui) = R(x(xi),u(ui));
		end
	end
	figure(10); clf(10)
	mesh(x,u,Rs')
	xlabel('x'); ylabel('u');zlabel('Penalty');	
end