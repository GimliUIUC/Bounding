function [x, z, th, dx, dz, dth] = decomposeX(X)

[m, n] = size(X);

if n == 1
    X = X';
end

x = X(:,1);
z = X(:,2);
th = X(:,3);
dx = X(:,4);
dz = X(:,5);
dth = X(:,6);

end