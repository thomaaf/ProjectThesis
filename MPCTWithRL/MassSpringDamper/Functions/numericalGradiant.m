function Jnum = numericalGradiant(MPCParam,Model,Gradiant,nums)
%% numericalGradiant(MPCParam,Model,Gradiant,nums)
%% MPCParam for parameters. Gradient for the symbolic equation. Nums for numerical values
%% All symbolic variables must be in MPCParam.
%% 
    [K,Plqr,e] = lqr(Model.A,Model.B,MPCParam.Q,MPCParam.R,[]);
    nums = reshape(nums',size(nums,1)*size(nums,2),1);
    vars = reshape(MPCParam.vars',size(MPCParam.vars,1)*size(MPCParam.vars,2),1);
    MPCvars = [reshape(MPCParam.Qsym,size(MPCParam.Qsym,1)*size(MPCParam.Qsym,2),1);
               reshape(MPCParam.Rsym,size(MPCParam.Rsym,1)*size(MPCParam.Rsym,2),1);
               reshape(MPCParam.fsym,size(MPCParam.fsym,1)*size(MPCParam.fsym,2),1);
               reshape(MPCParam.Psym,size(MPCParam.Psym,1)*size(MPCParam.Psym,2),1)];
    MPCnums = [reshape(MPCParam.Q,size(MPCParam.Q,1)*size(MPCParam.Q,2),1);
               reshape(MPCParam.R,size(MPCParam.R,1)*size(MPCParam.R,2),1);
               reshape(MPCParam.f,size(MPCParam.f,1)*size(MPCParam.f,2),1);
               reshape(Plqr,size(Plqr,1)*size(Plqr,2),1)];
    vars = [vars;MPCvars];
    nums = [nums;MPCnums];
    fstring = sprintf('MPCParam.F(');
    for i = 1:size(nums,1)
       fstring = sprintf('%s%.6e,',fstring,nums(i));
    end
    fstring = fstring(1:end-1); fstring = sprintf('%s)',fstring);
    Jnum = eval(eval(fstring));    
%    Jnum = eval(subs(Gradiant,vars,nums));
end



