function Analyze(x,u,t,out,MPCParam,RLParam,Model)
    figure(1)
    clf(1);
    pos = figure(1);
    subplot(3,1,1);
    plot(t(1:size(x,1),1),x(:,1),'Tag','Position'); grid on; 
    title('Position'); xlabel('Time [s]'); ylabel('Position [m]')
    axis([0 max(t) min(x(:,1))-1 max(x(:,1))+1]);
    
    
    subplot(3,1,2)
    plot(t(1:size(x,1),1),x(:,2),'Tag','Velocity'); grid on; 
    title('Velocity'); xlabel('Time [s]'); ylabel('Velocity [m/s]')
    axis([0 max(t) min(x(:,2))-1 max(x(:,2))+1]);

    subplot(3,1,3) 
    plot(t(1:size(x,1),1),u,'Tag','Input'); grid on; 
    title('Input'); xlabel('Time [s]'); ylabel('Force [N]')
    axis([0 max(t) min(u(:,1))-2 max(u(:,1))+2]);
    c = uicontrol(pos,'Style','slider','Position',[2 2 100 20],'Callback',@sliderUpdate);
    T = MPCParam.T; N = MPCParam.N;
    figure(2)
    plot(t(1:T/N/0.01:size(x,1),1),out.TD,'Tag','TD'); grid on; 
    title('Temporal Error'); xlabel('Time [s]'); 
    axis([0 max(t) min(out.TD(:,1))-1 max(out.TD(:,1))+1]);    
    
    figure(3)
    subplot(2,1,1)
    plot(t(1:T/N/0.01:size(x,1),1),out.theta(:,1:4),'Tag','Theta'); grid on; 
    axis([0 max(t) min(min(out.theta(:,1:4)))-1 max(max(out.theta(:,1:4)))+1]);  
    legend(string(RLParam.theta(1:4)),'Location','southeast','Orientation','vertical'...
        ,'LineWidth',0.1)    
    title("Convexity start: "+det(reshape(out.theta(end,1:4),2,2))+"  Convexity end: "+det(reshape(out.theta(1,1:4),2,2)))
    subplot(2,1,2)
    g  = plot(t(1:T/N/0.01:size(x,1),1),out.theta(:,5:8),'Tag','Theta'); grid on; 
    line([0 t(end)],[Model.A(1) Model.A(1) ],'Color',g(1).Color,'LineStyle','-.');
    line([0 t(end)],[Model.A(3) Model.A(3) ],'Color',g(2).Color,'LineStyle','-.');
    line([0 t(end)],[Model.A(2) Model.A(2) ],'Color',g(3).Color,'LineStyle','-.');
    line([0 t(end)],[Model.A(4) Model.A(4) ],'Color',g(4).Color,'LineStyle','-.');
    title("Stability start: "+det(reshape(out.theta(end,5:8),2,2))+"  Stability end: "+det(reshape(out.theta(1,5:8),2,2)))
    xlabel('Time [s]'); 
    
    legend(string(RLParam.theta(5:8)),'Location','southeast','Orientation','vertical'...
        ,'LineWidth',0.1)
    
    axis([0 max(t) min(min(out.theta(:,5:8)))-1 max(max(out.theta(:,5:8)))+1]);    
    
end

function sliderUpdate(src,event)
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
    subplot(3,1,1)
    m = string(t(round((n-1)*dt/0.01+1)));
    title('Position, optControl at t = '+m + " s")
    hold on;
    plot(t(trange), xopt(n,:),'b-.','Tag','optPos');
    plot(t(trange), xopt(n,:),'rx','Tag','optPosCross');
    hold off;
    
    subplot(3,1,2)
    hold on;
    plot(t(trange), xopt(n+size(xopt,1)/2,:),'b-.','Tag','optVel');
    plot(t(trange), xopt(n+size(xopt,1)/2,:),'rx','Tag','optVelCross');
    hold off;
    subplot(3,1,3)
    hold on;
    plot(t(round((n-1)*dt/0.01+1: dt/0.01: (n-1)*dt/0.01+1 + (T-T/N)/0.01)), uopt(n,:),'b-.','Tag','optInp');
    plot(t(round((n-1)*dt/0.01+1: dt/0.01: (n-1)*dt/0.01+1 + (T-T/N)/0.01)), uopt(n,:),'rx','Tag','optInpCross');
    hold off;
        
    

end