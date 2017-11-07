function [x, z, q1, dx, dz, dq1, s_b, s_f, t, T] = unpackState(X)

if(size(X,2)~= 10)
    X = X';
end

x = X(:,1);     z = X(:,2);     q1 = X(:,3);
dx = X(:,4);    dz = X(:,5);    dq1 = X(:,6);
s_b = X(:,7);   s_f = X(:,8);
t = X(:,9);     T = X(:,10);

end