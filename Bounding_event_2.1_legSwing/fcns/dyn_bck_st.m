function dXdt = dyn_bck_st(t,X,params,book,control)

p = params;
M = p.M;
g = p.g;
J = p.J;

Tst = control.Tst;
Tsw = control.Tsw;
alpha_z = control.alpha_z;
alpha_tau = control.alpha_tau;
coeff1 = control.coeff1;
coeff2 = control.coeff2;

x = X(1);
z = X(2);
th = X(3);
dx = X(4);
dz = X(5);
dth = X(6);
t_ = X(7);
s_f = X(8);
s_b = X(9);

% --- bookkeeping ---
t_TD = book.t_TD_b;

s_st = (t - t_TD)/Tst;
b = bezier(s_st,coeff1,coeff2);
tau = -alpha_tau * b;
Fz = alpha_z*b;

Fx = (tau + x*Fz)/z;

ddx = Fx/M;
ddz = Fz/M - g;
ddth = tau/J;
ds_f = 1/Tsw;
ds_b = 0;
f = [dx dz dth ddx ddz ddth 1 ds_f ds_b]';

dXdt = f;

end


