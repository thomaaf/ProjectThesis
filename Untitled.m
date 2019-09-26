clear;

n  = 4; 
V = zeros(1,n*n);
V2 = zeros(1,n*n);
% rewardGrid = [-1  -1  -1  -1   -10 -10;
%               -10 -10 2  -1   -10 -10;
%               -10 -10 2  -1   -10 -10;
%               -10 -10 2  -50   -10 -10;
%               -10 -10 2  -1   -10 -10;
%               -10 -10 -1  -1   -1  -1]

rewardGrid= [-1 -1 -1 -1;
             -1 -1 -1 -1;
             -1 -1 -10 -1;
             -1 -1 -1 -1];
rewardGrid = floor(randn(n)*4)
x0 = [3;3];
terminal1 = [1;1];
terminal2 = [n;n];
actionList = [];
posList = x0';
pos = x0;
gamma = 1;
c = 0;
actionSet = ["up","right","down","left"];
stateSet = [1,2,3;
            4,5,6;
            7,8,9];
for k = 1:200
	V=V2;
    for s = 1:n*n -1
        v = V(s);
        %disp("Next State")
        x = mod(s-1,n) + 1;
        y = floor((s-1)/n)+1;            
        %fprintf("Current Pos [%i , %i], with statenumber being %i\n",x,y,s);  
        val = 0;
        %% Policy iteration
%         for i = 1:4
%             a = actionSet(i);
%             sn = move(s,a);
%             xn = mod(sn-1,n) + 1;
%             yn = floor((sn-1)/n)+1;
%             %r = rewardGrid(x,y);
%             val = val + 0.25*1*(rewardGrid(xn,yn) + gamma*V(sn)); 
%         end
        %% Value iteration
        val = [];
        for i = 1:4
            a = actionSet(i);
            sn = move(s,a);
            xn = mod(sn-1,n) + 1;
            yn = floor((sn-1)/n)+1;
            %r = rewardGrid(x,y);
            val = [val; (rewardGrid(xn,yn) + 0.9*V(sn))]; 
        end
        val
        V2(s) = max(val);
        Vtmp = reshape(V2,n,n)
        
       
       
       %V(s) = 0.25*
       
    end
end
%%
round(reshape(V,n,n))
figure(1)
clf(1)
subplot(2,1,1)
surface(reshape(V,n,n) + rewardGrid); grid on;
xlabel('x'); ylabel('y')
subplot(2,1,2)
surf(rewardGrid)
xlabel('x'); ylabel('y')
function sn = move(s,a)
    n = evalin('base','n');
    x = mod(s-1,n) + 1;
    y = floor((s-1)/n)+1;     
    s = [x;y];
    if a == "up" && s(2) ~= 1
       sn = s - [0;1];
       sn = sn(1) + (sn(2)-1)*n;
    elseif a == "right" && s(1) ~= n
        sn = s + [1;0];
        sn = sn(1) + (sn(2)-1)*n;
    elseif a == "down" && s(2) ~= n
        sn = s + [0;1];  
        sn = sn(1) + (sn(2)-1)*n;
    elseif a == "left" && s(1) ~= 1
        sn = s - [1;0]; 
        sn = sn(1) + (sn(2)-1)*n;
    else   
        sn = s;
        sn = sn(1) + (sn(2)-1)*n;
    end
    x = mod(sn-1,n) + 1;
    y = floor((sn-1)/n)+1;    
    %fprintf("Going: %s\n",a);
    %fprintf("Next Pos [%i , %i], with statenumber being %i\n",x,y,sn);   
end

%% while (pos(1)~= terminal1(1) || pos(2)~= terminal1(2)) && (pos(1)~= terminal2(1) || pos(2)~= terminal2(2))
%%     
%     a = action();
%     s = pos;
%     [pos,actionList] = move(a,pos,n,actionList);
%     V = bellmannUpdate(s,pos,V);
%     posList = [posList;pos' ];
%     c = c +1;
% 
%     
% end
% 
% function V = bellmannUpdate(s,sn,V,n)
%     actions = ["up","right","left","down"];
%     val = 0;
%     for a = 1:4
%         [sn,pol] = moveCheck(actions(a), s,n);
%         val = val + pol*gamma*(-1 + V(sn(1),sn(2)));
%     end
%     V(s(1),s(2)) = val;
%     
% 
% end
% 
% function [sn,actionList] = move(a, s,n,actionList)
%     if a == "up" && s(2) ~= 1
%        sn = s - [0;1];
%        actionList = [actionList;1];
%     elseif a == "right" && s(1) ~= n
%         sn = s + [1;0];
%         actionList = [actionList;2];
%     elseif a == "down" && s(2) ~= n
%         sn = s + [0;1];
%         actionList = [actionList;3];
%     elseif a == "left" && s(1) ~= 1
%         sn = s - [1;0];
%         actionList = [actionList;4];
%     else   
%         sn = s;
%         actionList = [actionList;0];
%     end
% 
% end
% 
% function [sn,pol] = moveCheck(a, s,n)
%     if a == "up" && s(2) ~= 1
%         pol = 1;
%         sn = s - [0;1];
%     elseif a == "right" && s(1) ~= n
%         pol = 1;
%         sn = s + [1;0];
%     elseif a == "down" && s(2) ~= n
%         pol = 1;
%         sn = s + [0;1];
%     elseif a == "left" && s(1) ~= 1
%         pol = 1;
%         sn = s - [1;0];
%     else   
%         pol = 0;
%         sn = s;
%     end
% 
% end
% 
% function a = action()%Policy = uniform distribution
%     a = rand;
%     if a<=0.25
%         a = "up";
%     elseif (a >0.25)&&(a<=0.5)
%         a = "right";
%     elseif (a >0.5)&&(a<=0.75)
%         a = "down";
%     else 
%         a = "left";
%     end               
% end