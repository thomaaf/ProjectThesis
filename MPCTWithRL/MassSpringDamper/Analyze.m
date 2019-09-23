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
    N = evalin('base','N');T = evalin('base','T');
    xopt = evalin('base','xopt'); 
    uopt = evalin('base','uopt');
    t    = evalin('base','t');
    
    n = round(src.Value*size(uopt,1));
    optIndex = 1;
    for i= n:-1:1
        if ~isnan(xopt(i,1))
            optIndex = i;
            break;
        end
    end
    delete(findobj('tag','optPos'))
    delete(findobj('tag','optPosCross'))
    delete(findobj('tag','optVel'))
    delete(findobj('tag','optVelCross'))
    delete(findobj('tag','optInp'))
    delete(findobj('tag','optInpCross'))
    figure(1)
    subplot(3,1,1)
    m = string(t(optIndex));
    title('Position, optControl at t = '+m + " s")
    hold on;
    plot(t(optIndex:T/0.01/N:optIndex+T/0.01), xopt(optIndex,:),'b-.','Tag','optPos');
    plot(t(optIndex:T/0.01/N:optIndex+T/0.01), xopt(optIndex,:),'rx','Tag','optPosCross');
    hold off;
    
    subplot(3,1,2)
    hold on;
    plot(t(optIndex:T/0.01/N:optIndex+T/0.01), xopt(optIndex+size(xopt,1)/2,:),'b-.','Tag','optVel');
    plot(t(optIndex:T/0.01/N:optIndex+T/0.01), xopt(optIndex+size(xopt,1)/2,:),'rx','Tag','optVelCross');
    hold off;
    
    subplot(3,1,3)
    hold on;
    plot(t(optIndex:T/0.01/N:optIndex+T/0.01 -T/0.01/N), uopt(optIndex,:),'b-.','Tag','optInp');
    plot(t(optIndex:T/0.01/N:optIndex+T/0.01 -T/0.01/N), uopt(optIndex,:),'rx','Tag','optInpCross');
    hold off;
        
    

end