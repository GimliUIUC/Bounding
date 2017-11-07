function dXdt = dyn_frt_st(t,X,params,book,control)

p = params;
M = p.M;
g = p.g;
J = p.J;

Tst = control.Tst;
alpha_z = control.alpha_z;
alpha_tau = control.alpha_tau;
coeff1 = control.coeff1;
coeff2 = control.coeff2;

[x, z, th, dx, dz, dth] = decomposeX(X);

% --- bookkeeping ---
t_TD =    book.t_TD_f;
toe =     book.toe_f;

COM = [x;z];
r = toe - COM;

s_st = (t - t_TD)/Tst;
b = bezier(s_st,coeff1,coeff2);
tau = alpha_tau * b;
Fz = alpha_z*b;

Fx = (tau - r(1)*Fz)/r(2);

ddx = Fx/M;
ddz = Fz/M - g;
ddth = tau/J;
f = [dx dz dth ddx ddz ddth]';

dXdt = f;

end


