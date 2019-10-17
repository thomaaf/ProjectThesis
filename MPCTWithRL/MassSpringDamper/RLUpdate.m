function [TD, theta,L,delta] = RLUpdate(theta,Vs,Vsn,x,u,nabla,RLParam)
%% x is the current state xk. u is the current action ak
%% xn is the next state xk+1. un is the next action ak+1
    Q = [1 0; 0 1]; R = 1;
    gamma = RLParam.gamma; alfa = RLParam.alfa;
    L = x'* Q*x + u'*R*u;
    delta = gamma*Vsn - Vs;
    TD = L + delta;          %Rt+1 + y*V(sn) - Q(s,a)
	theta = (theta + alfa*TD*nabla.*RLParam.learn);  
    
end

% function L = Ltheta(x,u)
%     L = x'*x + u'*u;
% end

% TD = target + discounted function- function