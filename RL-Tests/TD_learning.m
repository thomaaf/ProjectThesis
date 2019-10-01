clear;

n  = 4; 
V = zeros(1,n*n);
E = zeros(1,n*n);
rewardGrid = -ones(n);
% rewardGrid = [-1  -1  -1  -1   -10 -10;
%               -10 -10 2  -1   -10 -10;
%               -10 -10 2  -1   -10 -10;
%               -10 -10 2  -50   -10 -10;
%               -10 -10 2  -1   -10 -10;
%               -10 -10 -1  -1   -1  -1]

% rewardGrid= [-1 -1 -1 -1;
%              -1 -1 -1 -1;
%              -1 -1 -10 -1;
%              -1 -1 -1 -1];

% rewardGrid = floor(randn(n)*4)


terminal1 = [1;1]; %rewardGrid(terminal1(1),terminal1(2)) = 0;
terminal2 = [n;n]; %rewardGrid(terminal2(1),terminal2(2)) = 0;

actionList = [];
gamma = .9;
alfa = 0.5;
lambda = 0.5;
actionSet = ["up","right","down","left"];
stateSet = [1,2,3;
            4,5,6;
            7,8,9];  
x0 = [floor(n/2);floor(n/2)];        
s = x0(1) + (x0(2)-1)*n;    
delta = [];
for k = 1:1000
    E = zeros(1,n*n);
    while s ~= 1 && s~=n*n
        CurrentPos = [mod(s-1,n) + 1;floor((s-1)/n)+1];
        %1. Take an action a given by pi,
        %       -Obsereve reward r and next state sn        
        a = chooseAction(s,V);
        sn = move(s,a); nextPos = [mod(sn-1,n) + 1;floor((sn-1)/n)+1];  
        r = rewardGrid(nextPos(1),nextPos(2));
        %2. Update the temporal difference
        delta = [delta;r + gamma*V(sn) - V(s)];
        %3. Update the eligibility trace of state s
        E(s) = E(s) + 1;
        
%         fprintf("Current position is  : [%i;%i]\nCurrent action is    : %s\n",CurrentPos(1),CurrentPos(2),a)
%         fprintf("next position becomes: [%i;%i]\n",nextPos(1),nextPos(2))
        
        %4. Sweep over all states s, and update the valueFunction
        for s = 1:n*n
            V(s) = V(s) + alfa*delta(end)*E(s);
            E(s) = gamma*lambda*E(s);
        end
        % Progress to next state
        s = sn;
        
        reshape(V,n,n)';
        reshape(E,n,n)';
    end

%     disp("Terminal")
    x0 = [floor(4*rand)+1;floor(4*rand)+1];        
    s = x0(1) + (x0(2)-1)*n;       
    
end
reshape(V,n,n)

%%
Dir = "";
u = zeros(1,n*n);
v = zeros(1,n*n);
for s = 1:n*n
    a = chooseAction(s,V);
    Dir = [Dir,a];
    s;
    
    if a == "up"
        u(s) = 0;
        v(s) = 1;
    elseif a == "right"
        u(s) = 1;
        v(s) = 0;
    elseif a == "down"
        u(s) = 0;
        v(s) = -1;
    else 
        u(s) = -1;
        v(s) = 0;
    end
    if s == 1 || s == n*n
        u(s) = 0;
        v(s) = 0;
        Dir(s+1) = ""
    end
    
end
Dir = Dir(2:end);
reshape(Dir,n,n)'
reshape(V,n,n)'     
u = reshape(u,n,n)';
v = reshape(v,n,n)';

figure(1)
hold on
clf(1)
for x = 1:n
    for y = 1:n
        hold on
        quiver(y,x,u(x,y),v(x,y));

        axis([0 n+1 0 n+1]);
    end
end
grid on
V = reshape(V,n,n)
surf(V)


%% Functions
function sn = move(s,a)
% Takes an action a, and spits out the next position given this action
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
end

function a = chooseAction(s,V)
    actionSet = ["up","right","down","left"];
	possibleNextPos = [move(s,'up');move(s,'right');move(s,'down');move(s,'left')];
    [value, argmax] = max(V(possibleNextPos));
    a = actionSet(argmax); 
end


 