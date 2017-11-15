
function dXdt = dyn_air(t,X,params,book,control)

g = params.g;

Tsw = control.Tsw;

dx = X(4);
dz = X(5);
dth = X(6);

ddx = 0;
ddz = -g;
ddth = 0;
ds_f = 1/Tsw;
ds_b = 1/Tsw;
f = [dx dz dth ddx ddz ddth 1 ds_f ds_b]';

dXdt = f;

end


