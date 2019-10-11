delete(findall(0,'Name','progress'))
fig = uifigure('Name', 'progress');
theta = [0.100139696759165,-3.927922697333582e-05,-3.927922697333582e-05,0.100017885876683,1.320639240468270e-05,0.999997403176278,-1.000001355368112,-0.999999439838899,1.358233477079862e-05,-1.450274924113488e-06];
t = 0.200000000000000;
syms q1 q2 q3 q4 real
syms a1 a2 a3 a4 real
syms e1 e2 real
RLParam.theta = [q1;q2;q3;q4;a1;a2;a3;a4;e1;e2];
x = [0.999487808187621;-0.014584986430379];
g = uigridlayout(fig,[3 1]);


timelabel = uilabel(g,'text',sprintf("Simulation time: % -3.3f\n",t));
timelabel.Layout.Row = 1;
timelabel.Layout.Column = 1;

statelabel = uilabel(g,'text',sprintf("Current State  : % -4.2f % -4.2f \n", x));
statelabel.Layout.Row = 2;
statelabel.Layout.Column = 1;
 


Parameterlabel = uilabel(g,'text', sprintf( ['Parameters     : ' repmat(' %-5s |', 1, size(RLParam.theta,1)) '\n'], RLParam.theta'));
Parameterlabel.Layout.Row = 3;
Parameterlabel.Layout.Column = 1;
% 
Parameternumlabel = uilabel(g,'text',sprintf( ['Parameters Val : ' repmat('% -6.4f  |', 1, size(theta,2)) '\n'], theta'));
Parameternumlabel.Layout.Row = 4;
Parameternumlabel.Layout.Column = 1;