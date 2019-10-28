function printState(theta_s  ,t,s,timelabel,statelabel,Parameternumlabel,TDs,a)
    A = [theta_s(5:6)';theta_s(7:8)'];
    Q = [theta_s(1:2)';theta_s(3:4)'];        
    timelabel.Text = sprintf("Simulation time: % -3.3f \t Temporal error: % -3.3f \t det(Q) = % -3.3f ",t,TDs,det(Q));
    statelabel.Text = sprintf("Current State  : % -4.2f % -4.2f \t Current Input: % -4.2f \t det(A) = % -3.2f", s,a,det(A));
    Parameternumlabel.Text = sprintf( ['Parameters Val : ' repmat('% -6.4f  |', 1, size(theta_s,1)) '\n'], theta_s');

end