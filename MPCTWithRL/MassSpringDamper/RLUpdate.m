function [TD, theta] = RLUpdate(theta,Vs,Vsn,x,u,nabla,R,Q,RLParam)
%% The rewardFunction is a quadratic function x^2 + u^2, as per example
%% x is the current state xk. u is the current action ak
%% xn is the next state xk+1. un is the next action ak+1
%% Meaning that Vsn is the result of the optimization, while we are at state x
    
    gamma = RLParam.gamma; alfa = RLParam.alfa;
    L = x'*Q*x + u'*R*u;
    TD = full(L + gamma*Vsn - Vs);          %R + y*V(sn) - Q(s,a)
	theta = (theta' + alfa*TD*nabla)';  
    
end

function L = Ltheta(x,u)
    L = x'*x + u'*u;
end
