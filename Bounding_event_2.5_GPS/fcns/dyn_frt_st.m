function dXdt = dyn_frt_st(t,X,params,book,control)

p = params;
M = p.M;
g = p.g;
J = p.J;
Kpth = p.Kpth;
Kdth = p.Kdth;
Kdx = p.Kdx;
Kpz = p.Kpz;
Kdz = p.Kdz;

Tst = control.Tst;
Tsw = control.Tsw;
alpha_z = control.alpha_z;
alpha_tau = control.alpha_tau;
coeff1 = control.coeff1;
coeff2 = control.coeff2;
zd = control.zd;
dzd = control.dzd;

x = X(1);
z = X(2);
th = X(3);
dx = X(4);
dz = X(5);
dth = X(6);
t_ = X(7);

% --- bookkeeping ---
t_TD =    book.t_TD_f;
s_st = (t - t_TD)/Tst;
b = bezier(s_st,coeff1,coeff2);

% --- forces & moment ---
tau = alpha_tau * b;
FHip = Kpz * (zd - z) + Kdz * (dzd - dz);
Fth = 1/x * (Kpth*(0 - th) + Kdth*(0 - dth));
Fz = alpha_z*b + FHip + Fth;
Fx = (tau + x*Fz)/z + Kdx*(0 - dx);

ddx = Fx/M;
ddz = Fz/M - g;
ddth = tau/J;
ds_f = 0;
ds_b = 1/Tsw;
dTst_ = 0;
f = [dx dz dth ddx ddz ddth 1 ds_f ds_b dTst_]';

dXdt = f;

end


