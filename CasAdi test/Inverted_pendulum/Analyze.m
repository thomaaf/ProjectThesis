function Analyze(x,t)
    A = [0,1,0,0;0,0,4.90500000000000,0;0,0,0,1;0,0,14.7150000000000,0];
    B = [0;1;0;2];
    Q = eye(4); R = 1; N = 0;
    K = lqr(A,B,Q,R,N);
    %u = x(5:8)*K';
    %x = [x, u];
    figure(1)
    clf(1)
    if(1)
    for i = 1:4:size(x,1)

       axis([-2,2,-1.5,1.5])
       title(t(i))
       line([x(i,1), x(i,1) - sin(x(i,3))],[0,cos(x(i,3))])
       hold on
       %line([x(i,5), x(i,5) - sin(x(i,7))],[0,cos(x(i,7))],'Color','black');
       
       grid on
       plot(x(i,1),0,'o')
       %plot(x(i,5),0,'x')
       pause(0.001)
%        if (abs(x(i,1))<0.01) && (abs(x(i,2))<0.01)
%            break;
%        end
       clf(1)
    end
    end
    name = ["Cart Pos","Cart Vel","Pendulum Pos","Pendulum Vel","input"];
    figure(2)
    clf(2)
    for i= 1:size(name,2)
       subplot(3,3,i)
       plot(x(:,i));
       hold on;
       plot(x(:,4+i))
       title(name(i))
       grid on;
    end
end

