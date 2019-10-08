function Analyze(x,u,t)
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