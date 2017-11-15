
function dXdt = dyn_air(t,X,params,book,control)

g = params.g;

dx = X(4);
dz = X(5);
dth = X(6);

ddx = 0;
ddz = -g;
ddth = 0;
f = [dx dz dth ddx ddz ddth 1]';


dXdt = f;

end


