clear;
%% TD(lambda)
% Implementation of TD(lambda) on a nxn gridworld. 

%% Enviroment 
% nxn gridworld with reward in each grid by R. R is randomized with -1 and
% -4 for dynamic paths. ~70% of R = -4
% Startpos is for storage of inital conditions in order for similar Traing
% sets. If desired, then R must be equal for each set, and defined as a
% matrix
n  =10; 
R = -ones(n);% R(1,3) = -3; R(3,1) = -3;
R = reshape(R',1,n*n);
%R = [-1,-1,-4,-1,-1,-4,-1,-4,-1,-1,-4,-1,-1,-1,-4,-1,-4,-1,-1,-1;-4,-1,-4,-1,-4,-4,-1,-1,-4,-1,-1,-4,-1,-4,-1,-4,-1,-4,-4,-1;-4,-1,-4,-1,-4,-4,-4,-4,-4,-4,-1,-1,-4,-1,-4,-1,-4,-4,-1,-4;-4,-4,-4,-4,-4,-1,-4,-4,-4,-1,-4,-4,-4,-4,-4,-4,-4,-4,-1,-1;-1,-4,-1,-4,-1,-1,-1,-4,-1,-1,-4,-1,-1,-1,-4,-4,-4,-1,-4,-1;-4,-1,-1,-4,-1,-4,-4,-4,-1,-1,-4,-1,-4,-4,-4,-1,-1,-1,-1,-1;-1,-4,-1,-1,-1,-1,-4,-4,-1,-4,-1,-4,-4,-1,-4,-4,-4,-4,-4,-4;-1,-1,-4,-1,-4,-4,-4,-4,-1,-4,-1,-4,-4,-4,-1,-1,-4,-1,-4,-1;-4,-4,-1,-4,-4,-1,-4,-1,-4,-4,-1,-4,-1,-1,-1,-4,-1,-4,-4,-1;-4,-4,-1,-4,-1,-1,-1,-4,-4,-1,-4,-4,-4,-4,-1,-4,-4,-1,-1,-1;-4,-1,-1,-1,-4,-4,-1,-1,-1,-1,-1,-4,-4,-4,-1,-4,-1,-4,-1,-4;-4,-4,-1,-1,-4,-1,-1,-1,-1,-1,-1,-1,-4,-1,-4,-4,-4,-4,-4,-1;-1,-1,-4,-4,-4,-1,-4,-1,-1,-1,-1,-1,-1,-1,-4,-1,-1,-4,-4,-1;-4,-1,-4,-1,-4,-1,-4,-4,-1,-1,-4,-1,-1,-1,-1,-4,-4,-1,-1,-4;-4,-1,-1,-1,-1,-4,-1,-1,-4,-1,-1,-1,-4,-1,-1,-4,-1,-1,-4,-1;-4,-4,-1,-1,-1,-1,-4,-4,-4,-1,-4,-1,-1,-4,-1,-4,-1,-4,-1,-1;-4,-4,-4,-4,-1,-1,-1,-4,-4,-1,-4,-1,-1,-1,-1,-1,-4,-4,-4,-1;-4,-4,-1,-1,-1,-1,-1,-1,-4,-1,-4,-1,-4,-1,-1,-1,-1,-4,-4,-1;-4,-1,-4,-4,-4,-1,-4,-1,-4,-1,-1,-1,-1,-1,-4,-1,-1,-1,-1,-1;-4,-1,-4,-1,-1,-1,-1,-4,-4,-4,-4,-1,-1,-4,-4,-4,-1,-4,-4,-1];
%R = [-1,-4,-4,-4,-4,-4,-1,-4,-1,-4;-1,-1,-1,-4,-4,-1,-4,-1,-4,-4;-1,-4,-1,-4,-1,-4,-4,-4,-4,-4;-1,-4,-4,-4,-1,-4,-1,-1,-1,-1;-1,-1,-4,-4,-1,-4,-4,-1,-1,-1;-1,-4,-1,-4,-4,-4,-1,-4,-4,-4;-4,-1,-4,-1,-1,-4,-4,-4,-1,-1;-1,-4,-1,-1,-1,-1,-4,-1,-1,-1;-4,-4,-4,-1,-4,-1,-1,-4,-4,-1;-4,-4,-4,-1,-1,-4,-1,-4,-4,-4];
for i = 1:n*n
    
    if rand>0.7
        R(i) = -4;
    end
end
x0 = [floor(n/2);floor(n/2)];        
s = x0(1) + (x0(2)-1)*n;   
nepisodes = 1000;

startpos = zeros(nepisodes,2);
%CurrentPos = [mod(s-1,n) + 1;floor((s-1)/n)+1];
%% RL-parameters

V = zeros(1,n*n);
E = zeros(1,n*n);
gamma =  .99;
alfa =   .8;
lambda = .8;
delta = [];

steps = zeros(nepisodes,1);

for k = 1:nepisodes           
    k
    while s ~= 1 && s~=n*n
        steps(k) = steps(k) + 1;
        
        a = chooseAction(s,V);                  %Taken action given by pi
        sn = move(s,a); 
        nextPos = [mod(sn-1,n) + 1;floor((sn-1)/n)+1];  
        r = R(s);                               %R(sn|s,a)
        delta = [delta;r + gamma*V(sn) - V(s)]; %TD-error
        E(s) = E(s) + 1;                        %Update trace of visited state
        for s = 1:n*n                           %Sweep over all positions 
            V(s) = V(s) + alfa*delta(end)*E(s);
            E(s) = gamma*lambda*E(s);
        end
        s = sn;                                 %Move to next position
    end
    
    x0 = [floor(n*rand)+1;floor(n*rand)+1];     %New random start postion
    startpos(k,1:2) = x0';                      %Store random start position
    %x0 = startpos(k,1:2)';
    s = x0(1) + (x0(2)-1)*n;       
    E = zeros(1,n*n);                            %Reset traces for next episode
end
%% Shape variables for readability, and write greedy path in dir

VRow = V; V = reshape(V,n,n)';
RRow = R; R = reshape(R,n,n)';
Dir = [];
u = zeros(1,n*n);
v = zeros(1,n*n);
for s = 1:n*n
    a = chooseAction(s,VRow);
    Dir = [Dir;a];
    if a == "up"
        v(s) = -1;
    elseif a == "right"
        u(s) = 1;
    elseif a == "down"
        v(s) =1;
    else 
        u(s) = -1;
    end
end
DirRow = Dir; Dir = reshape(Dir,n,n)';
uRow = u; u = reshape(u,n,n)';
vRow = v; v = reshape(v,n,n)';

%% Plott 

figure(1)
clf(1)
for x = 1:n
    for y = 1:n
        hold on;
        quiver(y,x,u(x,y),v(x,y),'k','lineWidth',2,'maxHeadSize',2)
        scatter(y,x,-R(x,y)*70,'r','filled')
    end
end
title("Greedy path with V-contour and scatter R")
xlabel('x'); ylabel('y'); grid on
contour(V,30)

figure(2)
clf(2)
plot(delta)
title("current TD-error")

movRMS = dsp.MovingRMS(200);
figure(3)
hold on
plot(movRMS(delta))
grid on;
title("TD-error RMS(200)")

figure(4)
surf(V + R)
xlabel('x'); ylabel('y'); grid on
title("Value + Rewards")
figure(5)
surf(R)
title("Rewards")
xlabel('x'); ylabel('y'); grid on
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


 