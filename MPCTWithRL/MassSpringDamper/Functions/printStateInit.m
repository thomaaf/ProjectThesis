function [timelabel,statelabel,Parameterlabel,Parameternumlabel]= printStateInit(theta,t,x,RLParam)
    delete(findall(0,'Name','progress'))
    fig = uifigure('Name', 'progress','Position',[680 866 1019 112]);
    
    g = uigridlayout(fig,[4 1]);
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
end
