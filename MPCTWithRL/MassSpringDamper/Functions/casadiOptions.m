function  [opts,args]= casadiOptions(Model,MPCParam,RLParam,InitParam)
% Redfining used parameters in order to reduce code size and increase
% readability
%Model
    nx = Model.nx; nu = Model.nu;
%MPC
    N = MPCParam.N;
    Xlb = MPCParam.Xlb; Xub = MPCParam.Xub;
    Ulb = MPCParam.Ulb; Uub = MPCParam.Uub; 
%% This function declares options for the solver (IPopt), and prepares other
%% arguments, such as bounds on states, for the Solver. 

%Options
    opts = struct;
    opts.ipopt.max_iter = 100;
    opts.ipopt.print_level = 0;
    opts.print_time = 0;
    opts.ipopt.acceptable_tol = 1e-8;
    opts.ipopt.acceptable_obj_change_tol = 1e-6;
    opts.calc_multipliers = 0; 
%Arguments
    args = struct;
% lbg&ubg = lower and upper bounds on the equality constraint
    args.lbg(1:nx*(N+1),1) = 0; 
    args.ubg(1:nx*(N+1),1) = 0;

%lbx&ubx = lower and upper bounds on the decision states;
%Checking for any upper or lower bounds, and adding them to args if
%existing.
    for i = 1:nx
        if ~isempty(Xlb) % Lower bounds on X
            args.lbx(i:nx:nx*(N+1),1) = Xlb(i); 
        else
            args.lbx(i:nx:nx*(N+1),1) = -inf; 
        end
        if ~isempty(Xub) % Upper bounds on X
            args.ubx(i:nx:nx*(N+1),1) = Xub(i);
        else
            args.ubx(i:nx:nx*(N+1),1) = inf;             
        end        
    end
    for i  = 1:nu
        if ~isempty(Ulb) % Lower bounds on U
            args.lbx(nx*(N+1)+i:nu:nx*(N+1) + nu*N,1) = Ulb(i); 
        else
            args.lbx(nx*(N+1)+i:nu:nx*(N+1) + nu*N,1) = -inf; 
        end
        if ~isempty(Uub) % Upper bounds on U
            args.ubx(nx*(N+1)+i:nu:nx*(N+1) + nu*N,1) = Uub(i); 
        else
            args.ubx(nx*(N+1)+i:nu:nx*(N+1) + nu*N,1) = inf;             
        end       
    end
    

end
