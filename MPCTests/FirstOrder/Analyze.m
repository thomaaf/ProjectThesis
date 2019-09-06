function Analyze(x,u,t,Xmpc,Umpc)
    A = [0,1,0,0;0,0,4.90500000000000,0;0,0,0,1;0,0,14.7150000000000,0];
    B = [0;1;0;2];
    Q = eye(4); R = 1; N = 0;
    K = lqr(A,B,Q,R,N);
    %u = x(5:8)*K';
    %x = [x, [0;u]];
    figure(1)
    clf(1)
    loopenabled = uicontrol('Style', 'pushbutton','String','hello','Callback',@BreakLoop,'Value',1);

    while(loopenabled.Value )
        pause(1)
        if ~loopenabled.Value 
            break;
        end
    for i = 1:4:size(x,1)
        if~loopenabled.Value
            break;
        end
       hold off
       %Cart
            plot(x(i,1)+0.2,0,'ob'); hold on;
            plot(x(i,1)-0.2,0,'ob')
            line([x(i,1)-0.2, x(i,1) + 0.2],[0,0])
            line([x(i,1)-0.2, x(i,1) + 0.2],[0.1,0.1])
            line([x(i,1)-0.2, x(i,1)-0.2],[0,0.1])
            line([x(i,1)+0.2, x(i,1)+0.2],[0,0.1])
       axis([-2,2,-1.5,1.5])
       title(t(i))       
       line([x(i,1), x(i,1) - sin(x(i,3))],[0.05,cos(x(i,3))])
       plot(x(i,1) - sin(x(i,3)),cos(x(i,3)),'ob')
       %line([x(i,5), x(i,5) - sin(x(i,7))],[0,cos(x(i,7))],'Color','black');
       grid on
       
       
   
%        %Cart
%             plot( Xmpc(i,1)+0.2,0,'or'); hold on;
%             plot( Xmpc(i,1)-0.2,0,'or')
%             line([Xmpc(i,1)-0.2, Xmpc(i,1) + 0.2],[0,0],'Color','red')
%             line([Xmpc(i,1)-0.2, Xmpc(i,1) + 0.2],[0.1,0.1],'Color','red')
%             line([Xmpc(i,1)-0.2, Xmpc(i,1)-0.2],[0,0.1],'Color','red')
%             line([Xmpc(i,1)+0.2, Xmpc(i,1)+0.2],[0,0.1],'Color','red')
%        line([Xmpc(i,1), Xmpc(i,1) - sin(Xmpc(i,3))],[0.05,cos(Xmpc(i,3))],'Color','red')
%        plot(Xmpc(i,1) - sin(Xmpc(i,3)),cos(Xmpc(i,3)),'or')
%        %line([x(i,5), x(i,5) - sin(x(i,7))],[0,cos(x(i,7))],'Color','black');
%        grid on
       
       %plot(x(i,5),0,'x')
       pause(0.001)
%         if (abs(x(i,1))<0.01) && (abs(x(i,2))<0.01)
%             break;
%         end
        
    end
    end
    name = ["Cart Pos","Cart Vel","Pendulum Pos","Pendulum Vel","input"];
    figure(2)
    %clf(2)
    for i= 1:size(name,2)
       subplot(2,3,i)
       hold on;
       plot(x(:,i));
       title(name(i))
       grid on;
    end
end

function BreakLoop(src,event)
    src.Value = 0
    event
end

