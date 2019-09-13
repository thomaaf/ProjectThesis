function V = MDP
t = 100;
maxAttempt = 100;

stageCost = zeros(t,maxAttempt );
x = zeros(t,maxAttempt ); x(1,:) = 20;
u = zeros(t,maxAttempt );
e = zeros(t,maxAttempt );
L = zeros(t,maxAttempt );
V = zeros(t,maxAttempt );
%figure(1); clf(1);
%figure(2); clf(2);
for attempt = 1:maxAttempt
    
    for i = 1:t
        u (i ,attempt) =  round(-x(i,attempt)/10);
        e(i  ,attempt)  = randn();
        L(i  ,attempt)  = 0.5*x(i,attempt)^2 + u(i,attempt)^2;
        x(i+1,attempt) =  x(i,attempt) + u(i,attempt) + e(i,attempt);
        %hold on; grid on
        %plot(x(i,attempt));
        
    end
    for i = 1:t
        sum = 0;
        gamma = 0.9;
        for k = i:t
            sum = sum + L(k,attempt)*gamma^k;

        end    
        V(i,attempt) = mean(sum);
    end
    n = 1;
    m = 3;
%     subplot(n,m,1)
%     hold on; grid on;
%     plot(x(:,attempt));
%     xlim([0;100])
%     
%     subplot(n,m,2)
%     hold on; grid on;
%     plot(L(:,attempt));
%     xlim([0;100])
%     
%     subplot(n,m,3)
%     hold on; grid on;
%     plot(V(:,attempt));
%     xlim([0;100])
end
V = mean(V(1,:));
end

function V = policyEvaluation()
    x = -20:1:20;
    policy = round(-x/10);
    L = 0.5*(x.^2 + policy.^2);
    V = zeros(size(x,1));
    gamma = 0.9;
    tol = 1;
    while norm(Vn - V)>tol;
        V = L + gamma*mean(V)
    end
end