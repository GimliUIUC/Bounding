%% one link float body dynamics for robot bounding
% Author: Yanran Ding
% last editted: 11/15/2017
function e = fcn_oneperiod(x_opt)

%%
params = ctrl_params;
p = params;
L = p.L;    l = p.l;    M = p.M;    J = p.J;    g = p.g;    l_leg = p.l_leg;
Tst = p.Tst;    
Tsw = 0.4;
T = Tst + Tsw;
Tair = (Tsw - Tst)/2;

coeff1 = p.st_co1;
coeff2 = p.st_co2;
c = mean(1/2 * coeff1 + 1/2 * coeff2);

%%
th0 = x_opt(1);
x0 = L/2*cos(th0);
z0 = L/2*sin(th0)+l_leg;
dx0 = x_opt(2);
dz0 = x_opt(3);
dth0 = x_opt(4);
s_f0 = Tair/Tsw;
s_b0 = 0;
% X = [x z th dx dz dth t_ph s_f s_b]
ic = [x0 z0 th0 dx0 dz0 dth0 0 s_f0 s_b0];
alpha_tau = 23;

%%
% coeff_x = [-0.1 -0.13 -0.1 -0.07  -0.02 0.02  0.07   0.1  0.13 0.1];
% coeff_z = [-0.35 -0.3 -0.2  -0.25 -0.32 -0.32 -0.25 -0.2 -0.3 -0.35];

% --- control variables ---

control.Tst = Tst;
control.Tsw = Tsw;
control.alpha_z = M*g*T/(2*c*Tst);
control.alpha_tau = alpha_tau;
control.coeff1 = coeff1;
control.coeff2 = coeff2;
control.zd = 0.2;
control.dzd = 0;
% control.coeff_x = coeff_x;
% control.coeff_z = coeff_z;
% control.foot = -1;      % -1 for hind leg, 1 for front leg

% --- bookkeeping ---
book.t_TD_f = 0;
book.t_LO_f = 0;
book.t_TD_b = 0;
book.t_LO_b = 0;

tstart = 0;
tfinal = 3;


tout = tstart;
Xout = ic;
teout = [];
Xeout = [];
ieout = [];


%% -------- back stance ---------
options = odeset('Events',@(t,X)eve_bck_LO(t,X,params,book,control),'MaxStep',1e-3);

[t,X,te,Xe,ie] = ode45(@(t,X)dyn_bck_st(t,X,params,book,control),[tstart, tfinal], ic,options);

nt = length(t);
tout = [tout;t(2:nt)];
Xout = [Xout;X(2:nt,:)];
teout = [teout;te];
Xeout = [Xeout;Xe];
ieout = [ieout;ie];
tstart = tout(end);

Xout(end,7) = 0;

%% --------- aerial phase --------
options = odeset('Events',@(t,X)eve_frt_TD(t,X,params,book,control),'MaxStep',1e-3);

[t,X,te,Xe,ie] = ode45(@(t,X)dyn_air(t,X,params,book,control),[tstart, tfinal], Xout(end,:),options);

nt = length(t);
tout = [tout;t(2:nt)];
Xout = [Xout;X(2:nt,:)];
teout = [teout;te];
Xeout = [Xeout;Xe];
ieout = [ieout;ie];
tstart = tout(end);

Xout(end,1) = -L/2*cos(X(end,3));    % reset x coordinate
Xout(end,7) = 0;
Xout(end,8) = 0;    % end of front swing

book.t_TD_f = te;
% book.toe_f(end+1) = X(end,1) + L/2*cos(X(end,3));

%% -------- front stance ---------
options = odeset('Events',@(t,X)eve_frt_LO(t,X,params,book,control),'MaxStep',1e-3);

[t,X,te,Xe,ie] = ode45(@(t,X)dyn_frt_st(t,X,params,book,control),[tstart, tfinal], Xout(end,:),options);

nt = length(t);
tout = [tout;t(2:nt)];
Xout = [Xout;X(2:nt,:)];
teout = [teout;te];
Xeout = [Xeout;Xe];
ieout = [ieout;ie];
tstart = tout(end);

Xout(end,7) = 0;

%% ------ second aerial phase ---------
options = odeset('Events',@(t,X)eve_bck_TD(t,X,params,book,control),'MaxStep',1e-3);

[t,X,te,Xe,ie] = ode45(@(t,X)dyn_air(t,X,params,book,control),[tstart, tfinal], Xout(end,:),options);

nt = length(t);
tout = [tout;t(2:nt)];
Xout = [Xout;X(2:nt,:)];
teout = [teout;te];
Xeout = [Xeout;Xe];
ieout = [ieout;ie];
tstart = tout(end);

Xout(end,1) = L/2*cos(X(end,3));    % reset x coordinate
Xout(end,7) = 0;
Xout(end,9) = 0;            % end of back swing

book.t_TD_b = te;
% book.toe_b(end+1) = X(end,1) - L/2*cos(X(end,3));

th_f = Xout(end,3);
dx_f = Xout(end,4);
dz_f = Xout(end,5);
dth_f = Xout(end,6);

error = x_opt - [th_f dx_f dz_f dth_f];

e = error * error';

end





