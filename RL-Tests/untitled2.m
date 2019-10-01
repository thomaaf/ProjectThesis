Dir = ""
for s = 1:n*n
    Dir = [Dir,chooseAction(s,V)];
end
Dir = Dir(2:end)
reshape(Dir,n,n)
%s = 10
%CurrentPos = [mod(s-1,n) + 1;floor((s-1)/n)+1]
%a = chooseAction(s,V)
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