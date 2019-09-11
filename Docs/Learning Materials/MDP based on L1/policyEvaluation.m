x = 10; u = -1;
%function V = policyEvaluation(x,u)
   
    V = 0; 
    L = 0.5*(x^2 + u^2);
    xnext = x + u + randn(100,1)
    
    for i = 1:200
       V 
        
        
        
    end
%end