figure(1)
clf(1)
for i =1:size(aset,1)
    k = 2; 
    col = [rand; rand;rand];
    while StateMatrix(aset(i)*2,k) ~= 0
        hold on;
        x = [StateMatrix(aset(i)*2-1,k -1); StateMatrix(aset(i)*2-1,k)];
        y = [StateMatrix(aset(i)*2  ,k -1); StateMatrix(aset(i)*2  ,k)];
        line(x,y,'Color',col)
        pos = plot(x(1),y(1),'rx');
        pause(0.1)
        delete(pos);
        title("Tot: "+size(aset,1)+"    Attempt: "+aset(i)+"   Step: "+k)
        axis([-1 6 -1 6]);
        grid on;
        k = k + 1;
    end 
    
end