clear
n = 3; m = 3;
Grid = -ones(n,m);

Grid(3,3) = 10; 
Grid (2,2) = -3; Grid(3,1) = -3;

a = zeros(1,4);
Q(1:n,1:m) = struct('a',a);

rate = 1;
piece = struct('stateX',1,'nextStateX',nan,'stateY',1,'nextStateY',nan,'action',0);
maxAttempts = 4000;
attempts = 1;
attemptData = struct('nSteps',zeros(maxAttempts,1),'stageCost',zeros(maxAttempts,1),'stageCostFiltered',zeros(maxAttempts,1));
stepCounter = 1;
StateMatrix = ones(2,1);
path = zeros(attempts, 20);
disp ("Start")
aset = [];
alfa = 0.1
while attempts <maxAttempts
    action = rand;
    explore = rand;

    if explore >.5
        [piece,action] = ranmove(piece,action,m,n);
        rate = 1;
    else
        [piece,action] = move(piece,optPolicy(Q,[piece.stateX;piece.stateY]),m,n);
        rate = 0;
    end
    if [piece.stateX ; piece.stateY] == [3;3]
       aset = [aset; attempts];
       piece.nextStateX = nan;
       piece.nextStateY = nan;
       disp("Finished")
       disp(attempts)
    end    
    
    %action
    if isnan(piece.nextStateY) || isnan(piece.nextStateX)
       attemptData.nSteps(attempts)= stepCounter;
       
       attempts = attempts + 1; 
       stepCounter = 1; 
       StateMatrix((attempts-1)*2 + 1:(attempts-1)*2 + 2, stepCounter ) = [1;1];
       piece = struct('stateX',1,'nextStateX',nan,'stateY',1,'nextStateY',nan,'action',0);
       %disp("================Out of bounds, next attempt=============")
    else
        stepCounter = stepCounter + 1;
        attemptData.stageCost(attempts) = attemptData.stageCost(attempts) + Grid(piece.nextStateY,piece.nextStateX);
        attemptData.stageCostFiltered(attempts+1) = attemptData.stageCostFiltered(attempts) + alfa*(...
            attemptData.stageCost(attempts) - attemptData.stageCostFiltered(attempts));
        Q = BellmanUpdate(Q,action,[piece.stateX;piece.stateY],[piece.nextStateX;piece.nextStateY],Grid,rate);
        piece.stateY = piece.nextStateY;
        piece.stateX = piece.nextStateX;
        StateMatrix((attempts-1)*2 + 1:(attempts-1)*2 + 2, stepCounter ) = [piece.stateX;piece.stateY];
    end
    
end
aset
figure(1)
subplot(2,1,1);
plot(attemptData.stageCost);
subplot(2,1,2)
plot(attemptData.stageCostFiltered)
function action = optPolicy(Q,s)
    best = -inf;
    for i= 1:4
        if Q(s(2),s(1)).a(i)> best
            best = Q(s(2),s(1)).a(i);
             action = i;
        end
    end
end


function Q = BellmanUpdate(Q,action,s,snext,Grid,rate)
    %Q = grid(n,m) with each grid having 4 actions
    %Action = 1:4
    %S = [X;Y], snext = [X;Y]
    %rate = 1;
    gamma = 0.9;
%     disp("Current position")
%     disp([s(2),s(1)])
%     disp("Current state")
%     disp(Q(s(2),s(1)).a)
%     disp("next state")
%     disp(Q(snext(2),snext(1)).a)
%     disp("Discount")
%     gamma*max(Q(snext(2),snext(1)).a)
    Q(s(2),s(1)).a(action) = Q(s(2),s(1)).a(action)+ rate*(Grid(snext(2),snext(1)) + gamma*max(Q(snext(2),snext(1)).a) - Q(s(2),s(1)).a(action));


end

function [piece,action] = move(piece, action,m,n)
    if action == 1                  %north
        disp("Moving North")
        if piece.stateY == 1
            piece.nextStateY = nan;
        else
            piece.nextStateY = piece.stateY - 1;
            piece.nextStateX = piece.stateX;            
        end
    elseif action == 2  %east
        disp("Moving East")
        if piece.stateX == m
            piece.nextStateX = nan;
        else
            piece.nextStateY = piece.stateY;
            piece.nextStateX = piece.stateX + 1;            
        end
    elseif action == 3 %south 
        disp("Moving South")
        if piece.stateY == n
            piece.nextStateY = nan;
        else
            piece.nextStateY = piece.stateY + 1;
            piece.nextStateX = piece.stateX;              
        end
          
    elseif action == 4                   %west
        disp("Moving West")
        if piece.stateX == 1
            piece.nextStateX = nan;
        else
            piece.nextStateY = piece.stateY;
            piece.nextStateX = piece.stateX - 1;                    
        end
    
    end


end


function [piece,action] = ranmove(piece, action,m,n)
    if action <= 0.25                  %north
        action = 1;
        %disp("Moving North")
        if piece.stateY == 1
            piece.nextStateY = nan;
        else
            piece.nextStateY = piece.stateY - 1;
            piece.nextStateX = piece.stateX;            
        end
    elseif action > 0.25 && action <= 0.5  %east
        action = 2;
        %disp("Moving East")
        if piece.stateX == m
            piece.nextStateX = nan;
        else
            piece.nextStateY = piece.stateY;
            piece.nextStateX = piece.stateX + 1;            
        end
    elseif action > 0.50 && action <= 0.75 %south 
        action = 3;
        %disp("Moving South")
        if piece.stateY == n
            piece.nextStateY = nan;
        else
            piece.nextStateY = piece.stateY + 1;
            piece.nextStateX = piece.stateX;              
        end
          
    elseif action > 0.75                   %west
        action = 4;
        %disp("Moving West")
        if piece.stateX == 1
            piece.nextStateX = nan;
        else
            piece.nextStateY = piece.stateY;
            piece.nextStateX = piece.stateX - 1;                    
        end
    
    end


end
