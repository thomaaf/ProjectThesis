function Analyze(x,u,t,out,MPCParam,RLParam,Model)
    row = 4; col = 3;
    figure(1)
    clf(1);
    pos = figure(1);    
    T = MPCParam.T; N = MPCParam.N;
    figtitle = sprintf("Alpha = %-6.6g, gamma = %-6.6g, tspan = %i",RLParam.alfa,RLParam.gamma,max(t(:)));
    figtit = uicontrol(pos,'Style','text','String',figtitle,'Units','normalized',...
       'Position',[.28 .95 .42 .05],'FontSize',13,'FontWeight','Bold');  
   %c = uicontrol(pos,'Style','slider','Position',[2 2 100 20],'Callback',@sliderUpdate);
%% System    

    subplot(row,col,1);
    plot(t(1:size(x,1),1),x(:,1),'Tag','Position'); grid on; 
    title('Position'); xlabel('Time [s]'); ylabel('Position [m]')
    axis([0 max(t) min(x(:,1))-1 max(x(:,1))+1]);
    
    inp2 = (out.u(2:end) - out.u(1:end-1))./(out.t(2:size(out.x,1),1) - out.t(1:size(out.x,1)-1,1));
    inp = zeros(size(u,1)-1,1);
    for i = 20:20:size(u,1)-1
        inp(i-19:i) = repmat(inp2(i),20,1);
    end
     inpEMA = zeros(size(u,1)-1,1);
     for i = 2:size(u,1)-1
        inpEMA(i) = inpEMA(i-1) + 0.0001*(inp(i) - inpEMA(i-1));
    end    
    moveRMS = dsp.MovingAverage(100);
    subplot(row,col,1)
    hold on;
    plot(t(1:size(x,1),1),x(:,2),'r','Tag','Velocity'); grid on; 
    %title('Velocity'); xlabel('Time [s]'); ylabel('Velocity [m/s]')
    %axis([0 max(t) min([min(x(:,1))-1,min(x(:,2))-1]) max([max(x(:,1))+1,max(x(:,2))+1])]);
    axis([0 max(t) min(-10)-1 max(10)+1]);    
    
    subplot(row,col,2) 
    plot(t(1:size(x,1),1),u,'Tag','Input'); grid on; hold on;
    plot(t(1:size(x,1)-1,1),inpEMA*5,'Tag','Input'); 
    title('Input'); xlabel('Time [s]'); ylabel('Force [N]')
    %axis([0 max(t) min(u(:,1))-2 max(u(:,1))+2]);
    axis([0 max(t) min(-10)-1 max(10)+1]);    

    %figure(9)

%% TD Error
    inp2 = (out.TD(2:end) - out.TD(1:end-1))./(out.t(2:size(out.x,1),1) - out.t(1:size(out.x,1)-1,1));
    inp = zeros(size(out.TD,1)-1,1);
    for i = 20:20:size(out.TD,1)-1
        inp(i-19:i) = repmat(inp2(i),20,1);
    end
     inpEMA = zeros(size(out.TD,1)-1,1);
     for i = 2:size(u,1)-1
        inpEMA(i) = inpEMA(i-1) + 0.0001*(inp(i) - inpEMA(i-1));
    end    
    subplot(row,col,3)
    plot(t(1:size(x,1),1),out.TD,'Tag','TD'); grid on; hold on;
    plot(t(1:size(x,1)-1,1),inpEMA*5,'Tag','TD'); 
    title('Temporal Error'); xlabel('Time [s]'); 
    %axis([0 max(t) min(out.TD(:,1))-1 max(out.TD(:,1))+1]);    
    axis([0 max(t) min(-10)-1 max(10)+1]);    
%% Theta(1:4)
    subplot(row,col,[4,5,6])
    plot(t(1:size(x,1),1),out.theta(:,1:4),'Tag','Theta'); grid on; 
    %axis([0 max(t) min(min(out.theta(:,1:4)))-1 max(max(out.theta(:,1:4)))+1]);
    axis([0 max(t) min(-10)-1 max(10)+1]);    
    legend(string(RLParam.theta(1:4)),'Location','southeast','Orientation','vertical'...
        ,'LineWidth',0.1)    
    title("Convexity start: "+det(reshape(out.theta(1,1:4),2,2))+"  Convexity end: "+det(reshape(out.theta(end,1:4),2,2)))
    
%% Theta(5:8)
    subplot(row,col,[7,8,9])
    g  = plot(t(1:size(x,1),1),out.theta(:,5:8),'Tag','Theta'); grid on; 
    line([0 t(end)],[Model.A(1) Model.A(1) ],'Color',g(1).Color,'LineStyle','-.');
    line([0 t(end)],[Model.A(3) Model.A(3) ],'Color',g(2).Color,'LineStyle','-.');
    line([0 t(end)],[Model.A(2) Model.A(2) ],'Color',g(3).Color,'LineStyle','-.');
    line([0 t(end)],[Model.A(4) Model.A(4) ],'Color',g(4).Color,'LineStyle','-.');
    title("Stability start: "+det(reshape(out.theta(1,5:8),2,2))+"  Stability end: "+det(reshape(out.theta(end,5:8),2,2)))
    xlabel('Time [s]'); 
    legend(string(RLParam.theta(5:8)),'Location','southeast','Orientation','vertical'...
        ,'LineWidth',0.1)
    %axis([0 max(t) min(min(out.theta(:,5:8)))-1 max(max(out.theta(:,5:8)))+1]);    
    axis([0 max(t) min(-10)-1 max(10)+1]);    
%% Theta(9:10)
    subplot(row,col,[10,11,12])
    plot(t(1:size(x,1),1),out.theta(:,9:10),'Tag','Theta'); grid on; 
    %axis([0 max(t) min(min(out.theta(:,9:10)))-1 max(max(out.theta(:,9:10)))+1]);
    axis([0 max(t) min(-10)-1 max(10)+1]);    
    legend(string(RLParam.theta(9:10)),'Location','southeast','Orientation','vertical'...
        ,'LineWidth',0.1)     
    title("E")
    
end

function sliderUpdate(src,event)
    row = 4; col = 3;
    out = evalin('base','out');
    %init = evalin('base','InitParam');
    MPCParam= evalin('base','MPCParam');
    N = MPCParam.N; T = MPCParam.T;
    xopt = out.xopt; 
    uopt = out.uopt;
    t = out.t;
%     N = evalin('base','N');T = evalin('base','T');
%     xopt = evalin('base','xopt'); 
%     uopt = evalin('base','uopt');
%     t    = evalin('base','t');
    dt = T/N;
    n = floor(src.Value*size(uopt,1)*.99+1);
    trange = round((n-1)*dt/0.01+1: dt/0.01: (n-1)*dt/0.01+1 + T/0.01);
    optIndex = 1;
    max(t-T)/(T/N);
    delete(findobj('tag','optPos'))
    delete(findobj('tag','optPosCross'))
    delete(findobj('tag','optVel'))
    delete(findobj('tag','optVelCross'))
    delete(findobj('tag','optInp'))
    delete(findobj('tag','optInpCross'))
    figure(1)
    subplot(row,col,1)
    m = string(t(round((n-1)*dt/0.01+1)));
    title('Position, optControl at t = '+m + " s")
    hold on;
    plot(t(trange), xopt(n,:),'b-.','Tag','optPos');
    plot(t(trange), xopt(n,:),'bx','Tag','optPosCross');
    hold off;
    
    subplot(row,col,1)
    hold on;
    plot(t(trange), xopt(n+size(xopt,1)/2,:),'r-.','Tag','optVel');
    plot(t(trange), xopt(n+size(xopt,1)/2,:),'rx','Tag','optVelCross');
    hold off;
    subplot(row,col,2)
    hold on;
    plot(t(round((n-1)*dt/0.01+1: dt/0.01: (n-1)*dt/0.01+1 + (T-T/N)/0.01)), uopt(n,:),'b-.','Tag','optInp');
    plot(t(round((n-1)*dt/0.01+1: dt/0.01: (n-1)*dt/0.01+1 + (T-T/N)/0.01)), uopt(n,:),'rx','Tag','optInpCross');
    hold off;
        
    

end