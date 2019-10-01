clear;

n  = 6; 
V = zeros(1,n*n);
V2 = zeros(1,n*n);
Vupdated = V;

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

rewardGrid = -ones(n); rewardGrid(2,2) = -2;
terminal1 = [1;1]; %rewardGrid(terminal1(1),terminal1(2)) = 0;
terminal2 = [n;n]; %rewardGrid(terminal2(1),terminal2(2)) = 0;

actionList = [];
gamma = .9;

actionSet = ["up","right","down","left"];
stateSet = [1,2,3;
            4,5,6;
            7,8,9];    
for k = 1:1000
    k-1;
    reshape(V,n,n);
    for s = 2:n*n -1 %For each state in s
        %disp("Next State")
        x = mod(s-1,n) + 1;
        y = floor((s-1)/n)+1;            
        %fprintf("Current Pos [%i , %i], with statenumber being %i\n",x,y,s);  
% All possible discounted values are stored
        valP = [];
        valV = [];
        for i = 1:4 % For each action a in A(s), 
            a = actionSet(i);   %For each action a possible in state s, 
            sn = move(s,a);     %For each next state s', given action a and state s
            xn = mod(sn-1,n) + 1;
            yn = floor((sn-1)/n)+1;
            
            valP = [valP; 1*(rewardGrid(xn,yn) + gamma*V(sn))]; 
            valV = [valV; 1*(rewardGrid(xn,yn) + gamma*V2(sn))];
        end
        % Policy evaluation: Update the value with the probability of
        % reacing each state, multiplied with the sum of the possible
        % states given all actions. As the probabilitiy is identical for
        % all actions, this simply becomes the sum of the four possible
        % outcomes of each action a, multiplied with 0.25
        V(s) = 0.25*sum(valP); 
        
        % Value iteration: Update the value with the maximum possible value
        % by each of the possible actions a. Aka, choose the value of the
        % discounted next state that results from choosing the greediest
        % action.  But isnt then this a other policy than the initial one?
        V2(s) = max((valV));

%% Progression


    end
end
reshape(V,n,n)

%%
(reshape(V,n,n))
(reshape(V2,n,n))
figure(1)
clf(1)
subplot(3,1,1)
surface(reshape(V,n,n) + rewardGrid); grid on;
xlabel('x'); ylabel('y')
subplot(3,1,2)
surface(reshape(V2,n,n) + rewardGrid); grid on;
xlabel('x'); ylabel('y')
subplot(3,1,3)
surf(rewardGrid)
xlabel('x'); ylabel('y')


%% Functions
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
end

 