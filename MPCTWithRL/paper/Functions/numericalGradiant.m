function [Jnum,numsi] = numericalGradiant(MPCParam,func,nums,Plqr,theta)
%% numericalGradiant(MPCParam,Model,Gradiant,nums)
%% MPCParam for parameters. Gradient for the symbolic equation. Nums for numerical values
%% All symbolic variables must be in MPCParam.
%% 
    
    

%     MPCnums = [%reshape(MPCParam.Q,size(MPCParam.Q,1)*size(MPCParam.Q,2),1);
%                reshape(MPCParam.R,size(MPCParam.R,1)*size(MPCParam.R,2),1);
%                reshape(MPCParam.f,size(MPCParam.f,1)*size(MPCParam.f,2),1);
%                reshape(Plqr,size(Plqr,1)*size(Plqr,2),1)];
    
    thetaNums = theta;
           
    numsi = [nums;thetaNums;reshape(Plqr,size(Plqr,1)*size(Plqr,2),1)];
    
    fstring = sprintf('MPCParam.%s(',func);
    for i = 1:size(numsi,1)
       fstring = sprintf('%s%.14e,',fstring,numsi(i));
    end
    fstring = fstring(1:end-1); fstring = sprintf('%s)',fstring);
    Jnum = eval(fstring);    
%    Jnum = eval(subs(Gradiant,vars,nums));
end



