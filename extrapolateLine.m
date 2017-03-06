function [x, y] = extrapolateLine(V)
%     keyboard;
    A = [V(:,3) ones(size(V,1),1)];
    beta = pinv(A)*V(:,1);
    
    x = flipud((0:1:500)');
    A = [x ones(size(x,1),1)];
    y = A*beta;
    

end