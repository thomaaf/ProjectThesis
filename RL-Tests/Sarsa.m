clear;
%% TD(lambda)
% Implementation of TD(lambda) on a nxn gridworld. 

%% Enviroment 
% nxn gridworld with reward in each grid by R. R is randomized with -1 and
% -4 for dynamic paths. ~70% of R = -4
% Startpos is for storage of inital conditions in order for similar Traing
% sets. If desired, then R must be equal for each set, and defined as a
% matrix
nx  =4; 
nu = 4;
R = -ones(nx);% R(1,3) = -3; R(3,1) = -3;
R = reshape(R',1,nx*nx);
%R = [-1,-1,-4,-1,-1,-4,-1,-4,-1,-1,-4,-1,-1,-1,-4,-1,-4,-1,-1,-1;-4,-1,-4,-1,-4,-4,-1,-1,-4,-1,-1,-4,-1,-4,-1,-4,-1,-4,-4,-1;-4,-1,-4,-1,-4,-4,-4,-4,-4,-4,-1,-1,-4,-1,-4,-1,-4,-4,-1,-4;-4,-4,-4,-4,-4,-1,-4,-4,-4,-1,-4,-4,-4,-4,-4,-4,-4,-4,-1,-1;-1,-4,-1,-4,-1,-1,-1,-4,-1,-1,-4,-1,-1,-1,-4,-4,-4,-1,-4,-1;-4,-1,-1,-4,-1,-4,-4,-4,-1,-1,-4,-1,-4,-4,-4,-1,-1,-1,-1,-1;-1,-4,-1,-1,-1,-1,-4,-4,-1,-4,-1,-4,-4,-1,-4,-4,-4,-4,-4,-4;-1,-1,-4,-1,-4,-4,-4,-4,-1,-4,-1,-4,-4,-4,-1,-1,-4,-1,-4,-1;-4,-4,-1,-4,-4,-1,-4,-1,-4,-4,-1,-4,-1,-1,-1,-4,-1,-4,-4,-1;-4,-4,-1,-4,-1,-1,-1,-4,-4,-1,-4,-4,-4,-4,-1,-4,-4,-1,-1,-1;-4,-1,-1,-1,-4,-4,-1,-1,-1,-1,-1,-4,-4,-4,-1,-4,-1,-4,-1,-4;-4,-4,-1,-1,-4,-1,-1,-1,-1,-1,-1,-1,-4,-1,-4,-4,-4,-4,-4,-1;-1,-1,-4,-4,-4,-1,-4,-1,-1,-1,-1,-1,-1,-1,-4,-1,-1,-4,-4,-1;-4,-1,-4,-1,-4,-1,-4,-4,-1,-1,-4,-1,-1,-1,-1,-4,-4,-1,-1,-4;-4,-1,-1,-1,-1,-4,-1,-1,-4,-1,-1,-1,-4,-1,-1,-4,-1,-1,-4,-1;-4,-4,-1,-1,-1,-1,-4,-4,-4,-1,-4,-1,-1,-4,-1,-4,-1,-4,-1,-1;-4,-4,-4,-4,-1,-1,-1,-4,-4,-1,-4,-1,-1,-1,-1,-1,-4,-4,-4,-1;-4,-4,-1,-1,-1,-1,-1,-1,-4,-1,-4,-1,-4,-1,-1,-1,-1,-4,-4,-1;-4,-1,-4,-4,-4,-1,-4,-1,-4,-1,-1,-1,-1,-1,-4,-1,-1,-1,-1,-1;-4,-1,-4,-1,-1,-1,-1,-4,-4,-4,-4,-1,-1,-4,-4,-4,-1,-4,-4,-1];
%R = [-1,-4,-4,-4,-4,-4,-1,-4,-1,-4;-1,-1,-1,-4,-4,-1,-4,-1,-4,-4;-1,-4,-1,-4,-1,-4,-4,-4,-4,-4;-1,-4,-4,-4,-1,-4,-1,-1,-1,-1;-1,-1,-4,-4,-1,-4,-4,-1,-1,-1;-1,-4,-1,-4,-4,-4,-1,-4,-4,-4;-4,-1,-4,-1,-1,-4,-4,-4,-1,-1;-1,-4,-1,-1,-1,-1,-4,-1,-1,-1;-4,-4,-4,-1,-4,-1,-1,-4,-4,-1;-4,-4,-4,-1,-1,-4,-1,-4,-4,-4];
% for i = 1:nx*nx
%     
%     if rand>0.7
%         R(i) = -4;
%     end
% end
x0 = [floor(nx/2);floor(nx/2)];        
s = x0(1) + (x0(2)-1)*nx;   
nepisodes = 1000;

startpos = zeros(nepisodes,2);
a = 1;
%CurrentPos = [mod(s-1,n) + 1;floor((s-1)/n)+1];

%% RL-parameters

Q = zeros(nx*nx,nu);
E = zeros(nx*nx,nu);
gamma =  .99;
alfa =   .8;
lambda = .2;
delta = [];

steps = zeros(nepisodes,1);

for k = 1:nepisodes         
    k
    while s ~= 1 && s~=1
        steps(k) = steps(k) + 1;                            %Store nSteps taken this episode
        sn = move(s,a);                                     %Take action a, observe sn,r
        r = R(s);                                           %R(sn|s,a)
        an = chooseAction(sn,Q);                            %Choose nex action a from policy & Q
        delta = [delta;r + gamma*Q(sn,an) - Q(s,a)];        %TD-error
        E(s,a) = E(s,a) + 1;                                %Update trace of visited state
        for s = 1:nx*nx                                     %Sweep over all states 
            for a = 1:nu                                    %Sweep over all actions at each state
                Q(s,a) = Q(s,a) + alfa*delta(end)*E(s,a);   %Update Action-value function
                E(s,a) = gamma*lambda*E(s,a);               %Update egilibit traces
            end
        end
        s = sn;                                 %Move to next position
        a = an;                                 %Ready action for next step
    end
    
    x0 = [floor(nx*rand)+1;floor(nx*rand)+1];     %New random start postion
    startpos(k,1:2) = x0';                      %Store random start position
    %x0 = startpos(k,1:2)';
    s = x0(1) + (x0(2)-1)*nx;       
    E = zeros(nx*nx,nu);                           %Reset traces for next episode
end
%% Shape variables for readability, and write greedy path in dir
Dir = zeros(1,nx*nx);
u   = zeros(1,nx*nx);
v   = zeros(1,nx*nx);
for s = 1:nx*nx
    [value, argmax] = max(Q(s,:));
    Dir(s) = argmax;
    if argmax == 1
        v(s) = 1;
    elseif argmax == 2
        u(s) = 1;
    elseif argmax == 3
        v(s) = -1;
    else 
        u(s) = -1;
    end
end
u = reshape(u,nx,nx)'
v = reshape(v,nx,nx)'
RRow = R; R = reshape(R,nx,nx)'
figure(1)
clf(1)
hold on
%scatter(1:nx,1:nx,RRow)
for y = 1:nx
    for x = 1:nx
        scatter(x,y,-R(x,y)*50,'fill')
        %quiver(x,y,u(x,y),v(x,y))
        
    end
end





%nextPos = [mod(sn-1,nx) + 1;floor((sn-1)/nx)+1];  
%% Functions
function sn = move(s,a)
% Takes an action a, and spits out the next position given this action
%Actionmap : [up,right,down,left] <=> [1,2,3,4]
    nx = evalin('base','nx');
    x = mod(s-1,nx) + 1;
    y = floor((s-1)/nx)+1;     
    s = [x;y];
    if a == 1 && s(2) ~= 1
       sn = s - [0;1];
       sn = sn(1) + (sn(2)-1)*nx;
    elseif a == 2 && s(1) ~= nx
        sn = s + [1;0];
        sn = sn(1) + (sn(2)-1)*nx;
    elseif a == 3 && s(2) ~= nx
        sn = s + [0;1];  
        sn = sn(1) + (sn(2)-1)*nx;
    elseif a == 4 && s(1) ~= 1
        sn = s - [1;0]; 
        sn = sn(1) + (sn(2)-1)*nx;
    else   
        sn = s;
        sn = sn(1) + (sn(2)-1)*nx;
    end
    x = mod(sn-1,nx) + 1;
    y = floor((sn-1)/nx)+1;    
    %fprintf("Going: %s\n",a);
end

function a = chooseAction(s,Q)
    %actionSet = ["up","right","down","left"];
    [value, argmax] = max(Q(s,:));
    a = argmax;
end





 