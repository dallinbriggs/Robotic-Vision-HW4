function [x, y] = extrapolateParabola(V)
%     keyboard;
    A = [V(:,3).^2 V(:,3) ones(size(V,1),1)];
    beta = pinv(A)*V(:,2);
    
    x = flipud((0:1:500)');
    A = [x.^2 x ones(size(x,1),1)];
    y = A*beta;
    

end