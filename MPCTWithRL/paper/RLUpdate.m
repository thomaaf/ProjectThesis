function [TD, theta,R,target] = RLUpdate(theta,Vs,Vsn,x,u,nabla,RLParam)
%% x is the current state xk. u is the current action ak
%% xn is the next state xk+1. un is the next action ak+1
    Q = RLParam.Q; R = RLParam.R;
    gamma = RLParam.gamma; alfa = RLParam.alfa;
    % Reward R for going to state s
    R = x'*Q*x + u'*R*u;
   
    target = R +1*gamma*Vsn;
    TD = target - Vs;          %Rt+1 + y*V(sn) - Q(s,a)
     
	%theta = (theta + alfa*(L + gamma*Vsn-Vs)*nabla.*RLParam.learn);  
    theta = (theta + alfa*TD*nabla.*RLParam.learn);  
    
end

% function L = Ltheta(x,u)
%     L = x'*x + u'*u;
% end

% TD = target + discounted function- function