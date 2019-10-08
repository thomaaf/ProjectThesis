function Jsym = symbolicGradiant(MPCParam,theta)
%% Calculates the gradient of the Lagrangian L, with respect to the symbolic variables
%% in theta. Theta should be a row vector
    rows = size(theta,1);
    Jsym(1:rows) = sym('0');
    L = sum(MPCParam.L);
    for i = 1:rows
        Jsym(i) = diff(L,theta(i),1);
    end
end