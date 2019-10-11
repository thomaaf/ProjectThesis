function [Jsym,Func] = symbolicGradiant(MPCParam,theta)
%% Calculates the gradient of the Lagrangian L, with respect to the symbolic variables
%% in theta. Theta should be a row vector
%% Also returns a function of the gradient. 
    rows = size(theta,1);
    Jsym(1:rows) = sym('0');
    L = sum(MPCParam.L);
    for i = 1:rows
        Jsym(i) = diff(L,theta(i),1);
    end
    Jsym = Jsym';
%     vars = reshape(MPCParam.vars',size(MPCParam.vars,1)*size(MPCParam.vars,2),1);
%     MPCvars = [reshape(MPCParam.Qsym,size(MPCParam.Qsym,1)*size(MPCParam.Qsym,2),1);
%                reshape(MPCParam.Rsym,size(MPCParam.Rsym,1)*size(MPCParam.Rsym,2),1);
%                reshape(MPCParam.fsym,size(MPCParam.fsym,1)*size(MPCParam.fsym,2),1);
%                reshape(MPCParam.Psym,size(MPCParam.Psym,1)*size(MPCParam.Psym,2),1)];
%     vars = [vars;MPCvars];           
    Func = symfun(Jsym,MPCParam.vars);
   Func = matlabFunction(Func);
end