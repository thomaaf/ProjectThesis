
n = 5; m = 5;
Grid = -ones(n,m);

Grid(5,5) = 10; 
Grid (5,2) = -2; Grid(4,3) = -2;

a = zeros(1,4);
%Q(1:n,1:m) = struct('a',a);

rate = 1;
piece = struct('stateX',1,'nextStateX',nan,'stateY',1,'nextStateY',nan,'action',0);
maxAttempts = 10;
attempts = 1;
attemptData(1:maxAttempts) = struct('nSteps',0);
stepCounter = 1;
StateMatrix = ones(2,1);
path = zeros(attempts, 20);
disp ("Start")


while attempts <maxAttempts
    action = rand;
    %[piece,action] = ranmove(piece,action,m,n);
    [piece,action] = move(piece,optPolicy(Q,[piece.stateX;piece.stateY]),m,n);
    %action
    if isnan(piece.nextStateY) || isnan(piece.nextStateX)
       attemptData(attempts).nSteps = stepCounter;
       
       attempts = attempts + 1; 
       stepCounter = 1; 
       StateMatrix((attempts-1)*2 + 1:(attempts-1)*2 + 2, stepCounter ) = [1;1];
       piece = struct('stateX',1,'nextStateX',nan,'stateY',1,'nextStateY',nan,'action',0);
       %disp("================Out of bounds, next attempt=============")
    else
        stepCounter = stepCounter + 1;
        piece.stateY = piece.nextStateY;
        piece.stateX = piece.nextStateX;
        %Q = BellmanUpdate(Q,action,[piece.stateX;piece.stateY],[piece.nextStateX;piece.nextStateY],Grid);
        StateMatrix((attempts-1)*2 + 1:(attempts-1)*2 + 2, stepCounter ) = [piece.stateX;piece.stateY]
    end
    
end

function action = optPolicy(Q,s)
    best = -inf;
    for i= 1:4
        if Q(s(2),s(1)).a(i)> best
            best = Q(s(2),s(1)).a(i);
             action = i;
        end
    end
end


function Q = BellmanUpdate(Q,action,s,snext,Grid)
    %Q = grid(n,m) with each grid having 4 actions
    %Action = 1:4
    %S = [X;Y], snext = [X;Y]
    rate = 1;
    
    rate*(Grid(s(2),s(1)) + max(Q(snext(2),snext(1)).a));
    Q(s(2),s(1)).a(action) = Q(s(2),s(1)).a(action)+ rate*(Grid(s(2),s(1)) + max(Q(snext(2),snext(1)).a) - Q(s(2),s(1)).a(action));


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
