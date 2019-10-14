function [TD, theta] = RLUpdate(theta,Vs,Vsn,x,u,nabla,RLParam)
%% x is the current state xk. u is the current action ak
%% xn is the next state xk+1. un is the next action ak+1
    Q = [1 0; 0 1]; R = 1;
    gamma = RLParam.gamma; alfa = RLParam.alfa;
    L = x'*Q*x + u'*R*u;
    TD = L + gamma*Vsn - Vs;          %R + y*V(sn) - Q(s,a)
	theta = (theta + alfa*TD*nabla);  
    
end

% function L = Ltheta(x,u)
%     L = x'*x + u'*u;
% end
