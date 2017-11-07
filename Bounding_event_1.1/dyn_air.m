
function dXdt = dyn_air(t,X,params,book,control)

g = params.g;

[x, z, th, dx, dz, dth] = decomposeX(X);

ddx = 0;
ddz = -g;
ddth = 0;
f = [dx dz dth ddx ddz ddth]';


dXdt = f;

end


